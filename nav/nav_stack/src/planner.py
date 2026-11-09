import rclpy
import time
import numpy as np
from scipy import ndimage
from math import ceil
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from nav2_msgs.msg import Costmap
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose

class NavPlanner(Node):
    def __init__(self):
        super().__init__('nav_planner')

        # Gets all the parameters from config specified in launch file
        self.declare_parameter('configuration.map_update_frequency', 5.0)
        self.declare_parameter('configuration.controller_update_frequency', 20.0)
        self.declare_parameter('configuration.robot_position_topic', '/odometry/global')
        self.declare_parameter('configuration.obstacle_map_topic', '/map')
        self.declare_parameter('configuration.path_map_topic', '/nav/planned_path')
        self.declare_parameter(
            'configuration.velocity_output_topic', '/nav_vel'
        )

        self.declare_parameter('mapping.inflation_radius', 1.0)
        self.declare_parameter('mapping.inflation_linear', False)
        self.declare_parameter('mapping.inflation_max', 100)
        self.declare_parameter('mapping.inflation_min', 20)
        self.declare_parameter('mapping.robot_radius', 1.0)
        self.declare_parameter('mapping.map_resolution', 1.0)

        self.declare_parameter('constraints.goal_tolerance', 1.0)
        self.declare_parameter(
            'constraints.max_sideways_linear_acceleration', 1.0
        )
        self.declare_parameter(
            'constraints.max_forwards_linear_acceleration', 1.0
        )
        self.declare_parameter('constraints.max_angular_acceleration', 1.0)
        self.declare_parameter('constraints.max_linear_velocity', 1.0)
        self.declare_parameter('constraints.max_angular_velocity', 1.0)

        self.declare_parameter('controller.lookahead_distance', 1.0)
        self.declare_parameter('controller.kP', 1.0)
        self.declare_parameter('controller.kI', 1.0)
        self.declare_parameter('controller.kD', 1.0)

        self.map_update_frequency = float(
            self.get_parameter('configuration.map_update_frequency').value
        )
        self.controller_update_frequency = float(
            self.get_parameter('configuration.controller_update_frequency').value
        )
        self.robot_position_topic = self.get_parameter(
            'configuration.robot_position_topic'
        ).value
        self.obstacle_map_topic = self.get_parameter(
            'configuration.obstacle_map_topic'
        ).value
        self.path_map_topic = self.get_parameter(
            'configuration.path_map_topic'
        ).value
        self.velocity_output_topic = self.get_parameter(
            'configuration.velocity_output_topic'
        ).value

        self.inflation_radius = float(
            self.get_parameter('mapping.inflation_radius').value
        )
        self.inflation_linear = bool(
            self.get_parameter('mapping.inflation_linear').value
        )
        self.inflation_max = int(
            self.get_parameter('mapping.inflation_max').value
        )
        self.inflation_min = int(
            self.get_parameter('mapping.inflation_min').value
        )
        self.robot_radius = float(self.get_parameter('mapping.robot_radius').value)
        self.map_resolution = float(
            self.get_parameter('mapping.map_resolution').value
        )

        self.goal_tolerance = float(
            self.get_parameter('constraints.goal_tolerance').value
        )
        self.max_sideways_linear_acceleration = float(
            self.get_parameter(
                'constraints.max_sideways_linear_acceleration'
            ).value
        )
        self.max_forwards_linear_acceleration = float(
            self.get_parameter(
                'constraints.max_forwards_linear_acceleration'
            ).value
        )
        self.max_angular_acceleration = float(
            self.get_parameter('constraints.max_angular_acceleration').value
        )
        self.max_linear_velocity = float(
            self.get_parameter('constraints.max_linear_velocity').value
        )
        self.max_angular_velocity = float(
            self.get_parameter('constraints.max_angular_velocity').value
        )

        self.lookahead_distance = float(
            self.get_parameter('controller.lookahead_distance').value
        )
        self.kP = float(self.get_parameter('controller.kP').value)
        self.kI = float(self.get_parameter('controller.kI').value)
        self.kD = float(self.get_parameter('controller.kD').value)

        # Creates variables to store current state
        self.costmap = None
        self.pose = None
        self.goal = None

        # Creates subscribers
        self.position_subscription = self.create_subscription(
            Odometry,
            self.robot_position_topic,
            self.position_update_callback,
            10)
        self.obstacle_subscription = self.create_subscription(
            OccupancyGrid,
            self.obstacle_map_topic,
            self.obstacle_update_callback,
            10)

        # Creates publishers
        self.publisher = self.create_publisher(
            Path,
            self.path_map_topic,
            10)
        self.costmap_publisher = self.create_publisher(
            Costmap,
            "/global_costmap/costmap",
            10)

        # Action server setup
        self.cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

    def obstacle_update_callback(self, msg):
        # Fixing resolution
        new_res = self.map_resolution
        old_res = msg.info.resolution
        old_w = msg.info.width
        old_h = msg.info.height
        
        original_grid = np.array(msg.data, dtype=np.int8).reshape((old_h, old_w))
        if old_res != new_res:
            scale_factor = old_res / new_res
            inflated_grid = ndimage.zoom(original_grid, scale_factor, order=0, prefilter=False)
        else:
            inflated_grid = original_grid.copy()
        inflated_grid = inflated_grid.astype(np.float32)
        new_h, new_w = inflated_grid.shape

        # Create masks for the two inflations
        obstacles_mask = (inflated_grid >= 100)
        lethal_cells = int(ceil(self.robot_radius / new_res))
        soft_cells = int(ceil(self.inflation_radius / new_res))

        # Adding No-Go zone based on robot radius
        y, x = np.ogrid[-lethal_cells:lethal_cells+1, -lethal_cells:lethal_cells+1]
        lethal_kernel = (x**2 + y**2 <= lethal_cells**2)
        lethal_zone = ndimage.binary_dilation(obstacles_mask, structure=lethal_kernel)

        # Adding discouragement based on inflation radius
        total_radius_cells = lethal_cells + soft_cells
        y_soft, x_soft = np.ogrid[-total_radius_cells:total_radius_cells+1, -total_radius_cells:total_radius_cells+1]
        soft_kernel = (x_soft**2 + y_soft**2 <= total_radius_cells**2)
        soft_zone = ndimage.binary_dilation(obstacles_mask, structure=soft_kernel)

        # Make changes into the grid
        free_space_mask = (inflated_grid >= 0) & (inflated_grid < 100)
        distance_from_obstacle = ndimage.distance_transform_edt(
            ~obstacles_mask,
            sampling=new_res,
        )
        distance_fraction = np.clip(
            (distance_from_obstacle - self.robot_radius)
            / self.inflation_radius,
            0.0,
            1.0,
        )

        if not self.inflation_linear:
            distance_fraction = 0

        inflation_values = np.rint(
            self.inflation_max
            + (self.inflation_min - self.inflation_max) * distance_fraction
        ).astype(np.int8)

        inflated_grid[soft_zone & free_space_mask] = inflation_values[
            soft_zone & free_space_mask
        ]

        inflated_grid[lethal_zone & free_space_mask] = np.inf

        # Create a costmap message compatible with Nav2 consumers.
        costmap = Costmap()
        costmap.header = msg.header
        costmap.metadata.map_load_time = msg.info.map_load_time
        costmap.metadata.update_time = self.get_clock().now().to_msg()
        costmap.metadata.layer = 'global_costmap'
        costmap.metadata.resolution = new_res
        costmap.metadata.size_x = int(new_w)
        costmap.metadata.size_y = int(new_h)
        costmap.metadata.origin = msg.info.origin

        cost_values = np.nan_to_num(
            inflated_grid,
            nan=0.0,
            posinf=255.0,
            neginf=0.0,
        )
        cost_values = np.clip(cost_values, 0.0, 255.0).astype(np.uint8)
        costmap.data = cost_values.flatten().tolist()

        self.costmap = costmap
        self.costmap_publisher.publish(costmap)
    
    def position_update_callback(self, msg):
        self.pose = msg


    def goal_callback(self, goal_request):
        """Called when the client sends a new goal."""
        self.get_logger().info(f'Received goal request to: '
                               f'({goal_request.pose.pose.position.x:.2f}, '
                               f'{goal_request.pose.pose.position.y:.2f})')
        # Accept the goal. This triggers the client's `_current_goal_handle.accepted` logic.
        self.goal = goal_request.pose.pose
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Called when the client triggers `cancel_goal_async()` (e.g., Ramp/Respawn)."""
        self.get_logger().info('Client requested to cancel the goal. Accepting cancellation.')
        # You must return ACCEPT, otherwise the client's cancel request is ignored.
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """The main execution loop for the navigation task."""
        self.get_logger().info('Executing goal...')
        
        result = NavigateToPose.Result()
        
        # Simulate navigation execution loop
        # In a real robot, you would monitor the robot's pose vs the goal pose here.
        for i in range(1, 11):
            # 1. Check if the client requested cancellation (respawn/ramp interrupt)
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled during execution!')
                goal_handle.canceled()
                return result
            
            # Simulate taking time to navigate (10 steps)
            self.get_logger().info(f'Navigating... {i * 10}% complete')
            time.sleep(0.5) 
            
        # 2. If we reach here without being canceled, the goal was successful
        self.get_logger().info('Goal reached successfully!')
        goal_handle.succeed()
        
        # 3. Return the result, triggering STATUS_SUCCEEDED in your client
        return result

def main(args=None):
    rclpy.init(args=args)
    node = NavPlanner()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()