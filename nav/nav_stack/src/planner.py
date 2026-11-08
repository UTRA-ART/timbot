import rclpy
import time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
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
        self.declare_parameter('configuration.robot_position_topic', '/tracked_pose')
        self.declare_parameter('configuration.robot_velocity_topic', '/odometry/local')
        self.declare_parameter('configuration.obstacle_map_topic', '/map')
        self.declare_parameter('configuration.path_map_topic', 'nav/planned_path')
        self.declare_parameter(
            'configuration.velocity_output_topic', '/twist_mux/cmd_vel'
        )

        self.declare_parameter('mapping.inflation_radius', 1.0)
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
        self.robot_velocity_topic = self.get_parameter(
            'configuration.robot_velocity_topic'
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
        # TODO

        # Creates subscribers
        self.position_subscription = self.create_subscription(
            PoseStamped,
            self.robot_position_topic,
            self.position_update_callback,
            10)
        self.velocity_subscription = self.create_subscription(
            PoseStamped,
            self.robot_velocity_topic,
            self.velocity_update_callback,
            10)
        self.obstacle_subscription = self.create_subscription(
            PoseStamped,
            self.obstacle_map_topic,
            self.obstacle_update_callback,
            10)

        # Creates publishers
        self.publisher = self.create_publisher(
            Path,
            self.path_map_topic,
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
        pass
    
    def position_update_callback(self, msg):
        pass

    def velocity_update_callback(self, msg):
        pass


    def goal_callback(self, goal_request):
        """Called when the client sends a new goal."""
        self.get_logger().info(f'Received goal request to: '
                               f'({goal_request.pose.pose.position.x:.2f}, '
                               f'{goal_request.pose.pose.position.y:.2f})')
        # Accept the goal. This triggers the client's `_current_goal_handle.accepted` logic.
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