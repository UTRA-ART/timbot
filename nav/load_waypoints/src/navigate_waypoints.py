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

from ament_index_python.packages import get_package_share_directory
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

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
        self.max_time_for_transform = 60.0
        self.waited_for_transform = False
        self.ignore_lidar = False
        self.start_direction = 1
        self.laps = 0
        self.current_lap = 0
        self.curr_waypoint_idx = 0
        self.result_received = 0
        self.ramp_naving = False
        self.cv_ramp_naving = th.Condition()

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.waypoint_pub = self.create_publisher(Bool, '/waypoint_int', 10)

        # Subscribers
        self.ramp_naving_sub = self.create_subscription(
            Bool, '/ramp_naving', self.ramp_naving_callback, 10
        )

        # Load waypoints
        self.populate_waypoint_dict()

        # Set initial waypoint index based on direction
        self.curr_waypoint_idx = 0 if self.start_direction == 1 else len(self.waypoints) - 2
        self.get_logger().info(f'First goal index: {self.curr_waypoint_idx}')

    def populate_waypoint_dict(self):
        """Load waypoints from JSON file."""
        try:
            with open(self.waypoints_file, 'r') as f:
                waypoint_data = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints file: {e}')
            sys.exit(1)

        self.start_direction = 1 if waypoint_data.get("start_direction", "north") == "north" else -1
        self.laps = waypoint_data.get("laps", 1)

        self.get_logger().info(f'Start direction: {"north" if self.start_direction == 1 else "south"}')
        self.get_logger().info(f'Laps: {self.laps}')

        # Wait for UTM transform (indicates GPS is ready)
        self.waited_for_transform = self.wait_for_utm_transform()

        gps_info = None
        if self.waited_for_transform:
            # Get initial GPS position
            gps_info = self.wait_for_gps_fix()
        else:
            self.get_logger().warn('Waiting for transform from /map to /utm timed out!')

        # Load waypoints
        if waypoint_data.get("add_corners", False) and gps_info:
            self.add_corners(waypoint_data, gps_info)
        else:
            for waypoint in waypoint_data["waypoints"]:
                self.waypoints[waypoint['id']] = waypoint

        # Append starting position as final waypoint (return home)
        if gps_info:
            last_idx = len(self.waypoints)
            self.waypoints[last_idx] = {
                'id': last_idx,
                'longitude': gps_info.longitude,
                'latitude': gps_info.latitude,
                'description': 'Initial start location',
                'frame_id': waypoint_data["waypoints"][0].get("frame_id", "map")
            }

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')

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

    def add_corners(self, waypoint_data, gps_info):
        """Add corner waypoints for better navigation."""
        frame = waypoint_data["waypoints"][0].get("frame_id", "map")
        j = 0

        for i in range(len(waypoint_data["waypoints"]) + 3):
            if i == 0:
                self.waypoints[i] = {
                    'id': i,
                    'longitude': gps_info.longitude,
                    'latitude': waypoint_data["waypoints"][0]["latitude"],
                    'description': "First Corner",
                    'frame_id': frame
                }
            elif i == 5:
                self.waypoints[i] = {
                    'id': i,
                    'longitude': waypoint_data["waypoints"][3]["longitude"],
                    'latitude': waypoint_data["waypoints"][3]["latitude"] - 0.000036,
                    'description': "Third Corner",
                    'frame_id': frame
                }
            elif i == 6:
                self.waypoints[i] = {
                    'id': i,
                    'longitude': gps_info.longitude,
                    'latitude': waypoint_data["waypoints"][3]["latitude"],
                    'description': "Fourth Corner",
                    'frame_id': frame
                }
            else:
                self.waypoints[i] = waypoint_data["waypoints"][j].copy()
                self.waypoints[i]["id"] = i
                j += 1

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
        """Get the next waypoint and update index."""
        waypoint = self.waypoints[self.curr_waypoint_idx]
        self.get_logger().info(f'Next Goal: {waypoint.get("description", self.curr_waypoint_idx)}')

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
            self.curr_waypoint_idx = len(self.waypoints) - 1
        elif self.curr_waypoint_idx >= len(self.waypoints) and self.current_lap < self.laps:
            self.current_lap += 1
            self.curr_waypoint_idx = 0

        return waypoint

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

    def send_goal_to_nav2(self, waypoint):
        """Send a goal to Nav2 and wait for result."""
        action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        if not action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available!')
            return False

        pose = self.get_pose_from_gps(
            waypoint["longitude"],
            waypoint["latitude"],
            waypoint.get("frame_id", "map")
        )

        if pose is None:
            self.get_logger().error('Could not transform waypoint to map frame')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f'Sending goal: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')

        future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')

        result_future = goal_handle.get_result_async()

        # Wait for result or ramp interrupt
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)

            with self.cv_ramp_naving:
                if self.ramp_naving:
                    self.get_logger().info('Navigation interrupted for ramp crossing')
                    goal_handle.cancel_goal_async()
                    self.cv_ramp_naving.wait_for(lambda: not self.ramp_naving)
                    self.get_logger().info('Resuming waypoint navigation')
                    return True  # Continue to next waypoint

        self.get_logger().info('Reached waypoint!')
        return True

    def navigate_waypoints(self):
        """Main navigation loop - iterates through all waypoints."""
        while rclpy.ok():
            if self.current_lap >= self.laps:
                self.get_logger().info('All laps completed!')
                break

            if self.curr_waypoint_idx < 0 or self.curr_waypoint_idx >= len(self.waypoints):
                self.get_logger().info('All waypoints visited!')
                break

            waypoint = self.get_next_waypoint()
            self.send_goal_to_nav2(waypoint)

    def ramp_naving_callback(self, msg):
        """Handle ramp navigation state changes."""
        with self.cv_ramp_naving:
            self.ramp_naving = msg.data
            if not self.ramp_naving:
                self.cv_ramp_naving.notify_all()


def main(args=None):
    rclpy.init(args=args)

    node = NavigateWaypoints()

    # Run navigation in separate thread
    nav_thread = th.Thread(target=node.navigate_waypoints)
    nav_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        nav_thread.join(timeout=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

            



