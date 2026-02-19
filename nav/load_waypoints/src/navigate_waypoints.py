#!/usr/bin/env python3
"""
Navigate Waypoints Node

Loads GPS waypoints from a JSON file and navigates through them sequentially
using Nav2. Coordinates with ramp_navigate for ramp crossing.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from ament_index_python.packages import get_package_share_directory
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs  # Required to register geometry_msgs types with tf2

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from nav2_msgs.action import NavigateToPose

import json
import sys
import time
import threading as th
import utm


class NavigateWaypoints(Node):
    def __init__(self):
        super().__init__('load_waypoints_server')

        # Get waypoints file from parameter (set by launch file)
        self.declare_parameter('waypoints_file', '')
        self.waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value

        if not self.waypoints_file:
            self.get_logger().error('No waypoints_file parameter provided!')
            sys.exit(1)

        self.get_logger().info(f'Loading waypoints from: {self.waypoints_file}')

        # State
        self.waypoints = dict()
        self.pose_queue = []  # Queue of (PoseStamped, description) tuples
        self.max_time_for_transform = 60.0
        self.waited_for_transform = False
        self.ignore_lidar = False
        self.start_direction = 1 # going clockwise (1)
        self.laps = 0
        self.current_lap = 0
        self.curr_waypoint_idx = 0
        self.result_received = 0
        self.ramp_naving = False
        self.cv_ramp_naving = th.Condition()
        self.goal_done_event = th.Event()
        self.goal_result = None

        # Callback group for action client (reentrant to allow concurrent callbacks)
        self.action_cb_group = ReentrantCallbackGroup()

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.waypoint_pub = self.create_publisher(Bool, '/waypoint_int', 10)

        # Subscribers
        self.ramp_naving_sub = self.create_subscription(
            Bool, '/ramp_naving', self.ramp_naving_callback, 10
        )

        # Load waypoints and convert to poses
        self.populate_waypoint_dict()

        # Set initial waypoint index based on direction
        self.curr_waypoint_idx = 0 if self.start_direction == 1 else len(self.pose_queue) - 2
        self.get_logger().info(f'First goal index: {self.curr_waypoint_idx}, Total waypoints: {len(self.pose_queue)}')

    def populate_waypoint_dict(self):
        """Load waypoints from JSON file and convert all to map-frame poses."""
        try:
            with open(self.waypoints_file, 'r') as f:
                waypoint_data = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints file: {e}')
            sys.exit(1)

        self.start_direction = 1 if waypoint_data.get("start_direction", "north") == "north" else -1
        self.laps = waypoint_data.get("laps", 1)

        self.get_logger().info(f'Start direction: {"East" if self.start_direction == 1 else "West"}')
        self.get_logger().info(f'Laps: {self.laps}')

        # Wait for UTM transform (indicates GPS is ready)
        self.waited_for_transform = self.wait_for_utm_transform()

        if not self.waited_for_transform:
            self.get_logger().warn('Waiting for transform from /map to /utm timed out!')
            return

        # Get initial GPS position and convert to UTM
        gps_info = self.wait_for_gps_fix()
        if not gps_info:
            self.get_logger().error('Failed to get GPS fix')
            return

        start_utm = utm.from_latlon(gps_info.latitude, gps_info.longitude)
        self.start_easting = start_utm[0]
        self.start_northing = start_utm[1]
        self.utm_zone = start_utm[2]
        self.utm_letter = start_utm[3]
        self.get_logger().info(f'Start UTM: ({self.start_easting:.2f}, {self.start_northing:.2f}) Zone {self.utm_zone}{self.utm_letter}')

        frame = waypoint_data["waypoints"][0].get("frame_id", "map")

        # Convert each waypoint lat/lon to UTM, then transform to map frame
        midpoint_utms = []
        for wp in waypoint_data["waypoints"]:
            utm_coords = utm.from_latlon(wp["latitude"], wp["longitude"], force_zone_number=self.utm_zone, force_zone_letter=self.utm_letter)
            midpoint_utms.append({
                'easting': utm_coords[0],
                'northing': utm_coords[1],
                'description': wp.get("description", f"Waypoint {wp['id']}")
            })
            self.get_logger().info(f'Midpoint {wp.get("description", wp["id"])}: UTM ({utm_coords[0]:.2f}, {utm_coords[1]:.2f})')

        # Build waypoint list with corners if requested
        if waypoint_data.get("add_corners", False):
            waypoint_utms = self.add_corners_utm(midpoint_utms)
        else:
            waypoint_utms = midpoint_utms

        # Convert all UTM coordinates to map-frame poses
        for i, wp_utm in enumerate(waypoint_utms):
            pose = self.utm_to_map_pose(wp_utm['easting'], wp_utm['northing'], frame)
            if pose:
                self.pose_queue.append((pose, wp_utm['description']))
                self.get_logger().info(f'Waypoint {i}: {wp_utm["description"]} -> ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')
            else:
                self.get_logger().warn(f'Failed to convert waypoint {i} to map frame')

        self.get_logger().info(f'Loaded {len(self.pose_queue)} waypoints into queue')

    def add_corners_utm(self, midpoints):
        """
        Add corner waypoints between midpoints using UTM coordinates.
        Expects 4 midpoints: East, South, West, North (in order)
        Robot starts facing east, so we go clockwise starting with SE corner.
        Returns: 8 waypoints (4 corners + 4 midpoints)
        """
        if len(midpoints) != 4:
            self.get_logger().warn(f'Expected 4 midpoints, got {len(midpoints)}. Skipping corners.')
            return midpoints

        result = []

        # lop to compute the midpoints:
        for i in range(4):
            curr = midpoints[i]
            next = midpoints[(i + 1) % 4]

            # Compute corner as a combination of current and next midpoints based on the direction of the turn
            corner_easting = curr['easting'] if self.start_direction == 1 else next['easting']
            corner_northing = next['northing'] if self.start_direction == 1 else curr['northing']
            result.append(curr)
            result.append({
                'easting': corner_easting,
                'northing': corner_northing,
                'description': f'Corner between {curr["description"]} and {next["description"]}'
            })
        
        # move the last point to the start
        result.insert(0, result.pop())
        return result

    def utm_to_map_pose(self, easting, northing, frame):
        """Convert UTM coordinates to a pose in the specified frame."""
        utm_pose = PoseStamped()
        utm_pose.header.frame_id = 'utm'
        utm_pose.header.stamp = self.get_clock().now().to_msg()
        utm_pose.pose.position.x = easting
        utm_pose.pose.position.y = northing
        utm_pose.pose.orientation.w = 1.0

        try:
            return self.tf_buffer.transform(utm_pose, frame, timeout=Duration(seconds=2))
        except Exception as e:
            self.get_logger().error(f'UTM to {frame} transform failed: {e}')
            return None
        
    def wait_for_gps_fix(self, timeout=10.0):
        """Wait for a GPS fix message."""
        gps_msg = None

        def callback(msg):
            nonlocal gps_msg
            gps_msg = msg

        sub = self.create_subscription(NavSatFix, '/gps/fix', callback, 10)

        start = time.time()
        while gps_msg is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.destroy_subscription(sub)
        return gps_msg

    def wait_for_utm_transform(self):
        """Wait for map->utm transform to become available (GPS ready)."""
        self.get_logger().info('Waiting for map->utm transform...')

        start_time = time.time()
        while (time.time() - start_time) < self.max_time_for_transform:
            try:
                self.tf_buffer.lookup_transform('map', 'utm', Time())
                self.get_logger().info(f'Transform found after {time.time() - start_time:.1f}s')
                return True
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)

        return False

    def get_next_waypoint(self):
        """Get the next waypoint pose and description, update index."""
        pose, description = self.pose_queue[self.curr_waypoint_idx]
        self.get_logger().info(f'Next Goal: {description} (idx {self.curr_waypoint_idx})')

        # Determine if we should ignore lidar (e.g., in certain segments)
        if self.curr_waypoint_idx == 3 and self.start_direction == 1:
            self.ignore_lidar = True
        elif self.curr_waypoint_idx == 2 and self.start_direction == -1:
            self.ignore_lidar = True
        else:
            self.ignore_lidar = False

        # Publish ignore lidar state
        msg = Bool()
        msg.data = self.ignore_lidar
        for _ in range(10):
            self.waypoint_pub.publish(msg)

        # Update index
        self.curr_waypoint_idx += self.start_direction
        if self.curr_waypoint_idx < 0 and self.current_lap < self.laps:
            self.current_lap += 1
            self.curr_waypoint_idx = len(self.pose_queue) - 1
        elif self.curr_waypoint_idx >= len(self.pose_queue) and self.current_lap < self.laps:
            self.current_lap += 1
            self.curr_waypoint_idx = 0

        return pose, description

    def get_pose_from_gps(self, longitude, latitude, frame):
        """Convert GPS coordinates to pose in specified frame."""
        utm_coords = utm.from_latlon(latitude, longitude)

        utm_pose = PoseStamped()
        utm_pose.header.frame_id = 'utm'
        utm_pose.header.stamp = self.get_clock().now().to_msg()
        utm_pose.pose.position.x = utm_coords[0]
        utm_pose.pose.position.y = utm_coords[1]
        utm_pose.pose.orientation.w = 1.0

        try:
            p_in_frame = self.tf_buffer.transform(utm_pose, frame, timeout=Duration(seconds=1))
            return p_in_frame
        except Exception as e:
            self.get_logger().error(f'Transform failed: {e}')
            return None

    def send_goal_to_nav2(self, pose, description):
        """Send a pre-converted pose to Nav2 and wait for result."""
        action_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose',
            callback_group=self.action_cb_group
        )

        if not action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available!')
            return False

        # Update timestamp on pose
        pose.header.stamp = self.get_clock().now().to_msg()

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f'Sending goal: {description} ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')

        self.goal_done_event.clear()
        self.goal_result = None
        self._current_goal_handle = None

        future = action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

        # Wait for goal acceptance
        start_time = time.time()
        while self._current_goal_handle is None and (time.time() - start_time) < 10.0:
            time.sleep(0.1)

        if self._current_goal_handle is None or not self._current_goal_handle.accepted:
            self.get_logger().error('Goal rejected or timed out')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')

        # Wait for result or ramp interrupt using event-based waiting
        while not self.goal_done_event.is_set():
            # Check periodically with short timeout
            if self.goal_done_event.wait(timeout=0.5):
                break

            with self.cv_ramp_naving:
                if self.ramp_naving:
                    self.get_logger().info('Navigation interrupted for ramp crossing')
                    if self._current_goal_handle:
                        self._current_goal_handle.cancel_goal_async()
                    self.cv_ramp_naving.wait_for(lambda: not self.ramp_naving)
                    self.get_logger().info('Resuming waypoint navigation')
                    return True  # Continue to next waypoint

        self.get_logger().info('Reached waypoint!')
        return True

    def _goal_response_callback(self, future):
        """Callback when goal response is received."""
        self._current_goal_handle = future.result()
        if self._current_goal_handle.accepted:
            result_future = self._current_goal_handle.get_result_async()
            result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """Callback when goal result is received."""
        self.goal_result = future.result()
        self.goal_done_event.set()

    def navigate_waypoints(self):
        """Main navigation loop - iterates through all waypoints in pose_queue."""
        if not self.pose_queue:
            self.get_logger().error('No waypoints loaded!')
            return

        while rclpy.ok():
            if self.current_lap >= self.laps:
                self.get_logger().info('All laps completed!')
                break

            if self.curr_waypoint_idx < 0 or self.curr_waypoint_idx >= len(self.pose_queue):
                self.get_logger().info('All waypoints visited!')
                break

            pose, description = self.get_next_waypoint()
            self.send_goal_to_nav2(pose, description)

    def ramp_naving_callback(self, msg):
        """Handle ramp navigation state changes."""
        with self.cv_ramp_naving:
            self.ramp_naving = msg.data
            if not self.ramp_naving:
                self.cv_ramp_naving.notify_all()


def main(args=None):
    rclpy.init(args=args)

    node = NavigateWaypoints()

    # Use MultiThreadedExecutor to handle callbacks from background thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Run navigation in separate thread
    nav_thread = th.Thread(target=node.navigate_waypoints, name='navigate_waypoints')
    nav_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nav_thread.join(timeout=5.0)
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

            



