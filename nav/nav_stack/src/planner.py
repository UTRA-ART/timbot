#!/usr/bin/env python3

import rclpy
import time
import heapq
import math
import numpy as np
from scipy import ndimage
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
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
    
        self.declare_parameter('mapping.obstacle_threshold', 50)
        self.declare_parameter('mapping.inflation_radius', 1.0)
        self.declare_parameter('mapping.inflation_linear', False)
        self.declare_parameter('mapping.inflation_max', 100)
        self.declare_parameter('mapping.inflation_min', 20)
        self.declare_parameter('mapping.robot_radius', 1.0)
        self.declare_parameter('mapping.map_resolution', 1.0)
        self.declare_parameter('planning.curvature_cost_weight', 1.0)
        self.declare_parameter('planning.max_search_distance', 20.0)

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

        self.obstacle_threshold = float(
            self.get_parameter('mapping.obstacle_threshold').value
        )
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
        self.curvature_cost_weight = float(
            self.get_parameter('planning.curvature_cost_weight').value
        )
        self.max_search_distance = float(
            self.get_parameter('planning.max_search_distance').value
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
        costmap_qos = QoSProfile(depth=1)
        costmap_qos.reliability = ReliabilityPolicy.RELIABLE
        costmap_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.costmap_publisher = self.create_publisher(
            OccupancyGrid,
            "/global_costmap/costmap",
            costmap_qos)

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
        self.get_logger().info(
            'NavPlanner started; listening for goals on /navigate_to_pose'
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

        obstacles_mask = (inflated_grid >= self.obstacle_threshold)
        free_space_mask = (inflated_grid >= 0) & (inflated_grid < 100)
        inflated_grid = np.full_like(inflated_grid, -1)
        

        # Make changes into the grid
        if np.any(obstacles_mask):
            distance_from_obstacle = ndimage.distance_transform_edt(
                ~obstacles_mask,
                sampling=new_res,
            )
            lethal_zone = distance_from_obstacle <= self.robot_radius
            soft_zone = distance_from_obstacle <= (
                self.robot_radius + self.inflation_radius
            )
            distance_fraction = np.clip(
                (distance_from_obstacle - self.robot_radius)
                / self.inflation_radius,
                0.0,
                1.0,
            )

            if not self.inflation_linear:
                distance_fraction = np.zeros_like(distance_from_obstacle)

            inflation_values = np.rint(
                self.inflation_max
                + (self.inflation_min - self.inflation_max) * distance_fraction
            ).astype(np.int8)

            inflated_grid[soft_zone & free_space_mask] = inflation_values[
                soft_zone & free_space_mask
            ]
            inflated_grid[lethal_zone & free_space_mask] = 100
            inflated_grid[obstacles_mask] = 100

        # Create an OccupancyGrid so RViz's Map display can render the costmap.
        costmap = OccupancyGrid()
        costmap.header = msg.header
        costmap.info.map_load_time = msg.info.map_load_time
        costmap.info.resolution = new_res
        costmap.info.width = int(new_w)
        costmap.info.height = int(new_h)
        costmap.info.origin = msg.info.origin

        cost_values = np.nan_to_num(
            inflated_grid,
            nan=-1.0,
            posinf=100.0,
            neginf=0.0,
        )
        cost_values = np.clip(cost_values, 0.0, 100.0).astype(np.int8)
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

    def _world_to_grid(self, x, y, costmap):
        origin = costmap.info.origin.position
        return (
            int(math.floor((x - origin.x) / costmap.info.resolution)),
            int(math.floor((y - origin.y) / costmap.info.resolution)),
        )

    def _grid_to_world(self, x, y, costmap):
        origin = costmap.info.origin.position
        resolution = costmap.info.resolution
        return (
            origin.x + (x + 0.5) * resolution,
            origin.y + (y + 0.5) * resolution,
        )

    def _find_path(self, start, goal, costmap):
        """Find a collision-free path in the current costmap with A*."""
        width = int(costmap.info.width)
        height = int(costmap.info.height)
        costs = np.asarray(costmap.data, dtype=np.int16).reshape((height, width))
        start_x, start_y = start
        goal_x, goal_y = goal

        if not (0 <= start_x < width and 0 <= start_y < height
                and 0 <= goal_x < width and 0 <= goal_y < height):
            return None

        # A cost of 100 is lethal; -1 is unknown and is not safe to traverse.
        traversable = (costs >= 0) & (costs < 100)
        if not traversable[start_y, start_x] or not traversable[goal_y, goal_x]:
            return None

        resolution = float(costmap.info.resolution)
        distance_to_goal = math.hypot(
            (goal_x - start_x) * resolution,
            (goal_y - start_y) * resolution,
        )
        search_radius = max(distance_to_goal, self.max_search_distance)
        search_radius_squared = search_radius ** 2
        directions = (
            (-1, -1), (0, -1), (1, -1),
            (-1, 0),            (1, 0),
            (-1, 1),  (0, 1),  (1, 1),
        )
        def heuristic(x, y):
            return math.hypot(goal_x - x, goal_y - y)

        # Include the incoming direction in the state so a turn penalty is
        # evaluated without making the path cost dependent on search order.
        start_state = (start_x, start_y, -1)
        open_set = [(heuristic(start_x, start_y), 0.0, start_state)]
        best_cost = {start_state: 0.0}
        came_from = {}

        while open_set:
            _, current_cost, current = heapq.heappop(open_set)
            if current_cost > best_cost.get(current, float('inf')):
                continue

            x, y, incoming_direction = current
            if (x, y) == (goal_x, goal_y):
                states = [current]
                while states[-1] != start_state:
                    states.append(came_from[states[-1]])
                states.reverse()
                return [(state[0], state[1]) for state in states]

            for direction_index, (dx, dy) in enumerate(directions):
                next_x, next_y = x + dx, y + dy
                if not (0 <= next_x < width and 0 <= next_y < height):
                    continue
                if not traversable[next_y, next_x]:
                    continue

                offset_x = (next_x - start_x) * resolution
                offset_y = (next_y - start_y) * resolution
                if offset_x ** 2 + offset_y ** 2 > search_radius_squared:
                    continue

                # Do not let a diagonal step cut through the corner of an
                # obstacle.
                if dx and dy and (
                    not traversable[y, next_x] or not traversable[next_y, x]
                ):
                    continue

                step_distance = math.hypot(dx, dy)
                cell_cost = float(costs[next_y, next_x]) / 100.0
                turn_cost = 0.0
                if incoming_direction >= 0:
                    previous_dx, previous_dy = directions[incoming_direction]
                    dot = previous_dx * dx + previous_dy * dy
                    previous_length = math.hypot(previous_dx, previous_dy)
                    turn_cost = self.curvature_cost_weight * (
                        1.0 - dot / (previous_length * step_distance)
                    )

                next_state = (
                    next_x,
                    next_y,
                    direction_index,
                )
                next_cost = current_cost + step_distance + cell_cost + turn_cost
                if next_cost >= best_cost.get(next_state, float('inf')):
                    continue
                best_cost[next_state] = next_cost
                came_from[next_state] = current
                heapq.heappush(
                    open_set,
                    (next_cost + heuristic(next_x, next_y), next_cost, next_state),
                )

        return None

    def _path_message(self, grid_path, costmap, goal, start_position):
        path = Path()
        path.header.frame_id = costmap.header.frame_id or 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        for index, (grid_x, grid_y) in enumerate(grid_path):
            x, y = self._grid_to_world(grid_x, grid_y, costmap)
            if index == 0:
                x, y = start_position.x, start_position.y
            elif index == len(grid_path) - 1:
                x, y = goal.position.x, goal.position.y
            if index + 1 < len(grid_path):
                next_x, next_y = self._grid_to_world(
                    grid_path[index + 1][0], grid_path[index + 1][1], costmap
                )
                if index + 1 == len(grid_path) - 1:
                    next_x, next_y = goal.position.x, goal.position.y
                yaw = math.atan2(next_y - y, next_x - x)
            else:
                yaw = 2.0 * math.atan2(goal.orientation.z, goal.orientation.w)

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        return path

    def execute_callback(self, goal_handle):
        """The main execution loop for the navigation task."""
        self.get_logger().info('Executing goal...')
        
        result = NavigateToPose.Result()
        
        # Simulate navigation execution loop
        # In a real robot, you would monitor the robot's pose vs the goal pose here.

        refreshing = False
        target_goal = self.goal

        while True:
            # 1. Check if the client requested cancellation (respawn/ramp interrupt)
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled during execution!')
                goal_handle.canceled()
                return result

            planned_path = Path()
            # If the robot is at the target
            condition_to_success = False
            if self.pose is not None and target_goal is not None:
                robot_position = self.pose.pose.pose.position
                goal_position = target_goal.position
                goal_distance = np.hypot(
                    goal_position.x - robot_position.x,
                    goal_position.y - robot_position.y,
                )
                condition_to_success = goal_distance <= self.goal_tolerance

            done_refreshing = False
            if condition_to_success:
                if refreshing:
                    refreshing = False
                    done_refreshing = True
                    target_goal = self.goal
                else:
                    self.publisher.publish(planned_path)
                    self.get_logger().info('Goal reached successfully!')
                    goal_handle.succeed()
                    break

            path_found = False
            if self.costmap is not None and self.pose is not None and target_goal is not None:
                robot_position = self.pose.pose.pose.position
                start = self._world_to_grid(
                    robot_position.x, robot_position.y, self.costmap
                )
                goal = self._world_to_grid(
                    target_goal.position.x, target_goal.position.y, self.costmap
                )
                grid_path = self._find_path(start, goal, self.costmap)
                if grid_path is not None:
                    planned_path = self._path_message(
                        grid_path, self.costmap, target_goal, robot_position
                    )
                    path_found = True

            # Does not need to consider turning rates / acceleration since a differential drive can do this.
            # Instead, we can add a cost that includes curvature so less curvature is better inside regular A*
            # We also need to include a max search distance, limits the field to only consider a circle centered at the robot to the goal

            # TODO

            # If a path can't be computed
            if (not path_found):
                if refreshing or done_refreshing:
                    pass # Clear costmap
                else:
                    # Find a target goal in the opposite direction
                    target_goal = self.goal
                    refreshing = True

                    # self.publisher.publish(planned_path)
                    self.get_logger().info('Path not found')
                    break

            # Publish path
            # self.get_logger().info('Planning')
            self.publisher.publish(planned_path)            
            time.sleep(0.1)
            
        # Return the result after the goal has been marked successful.
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