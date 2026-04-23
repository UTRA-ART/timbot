#!/usr/bin/env python3
"""
Ramp Navigation Node

Detects ramps from /ramp_seg topic and takes over navigation to safely cross them.
Publishes /ramp_naving to pause waypoint navigation while crossing.

State machine: no_ramp -> to_ramp -> on_ramp -> no_ramp
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
import numpy as np
import math

from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose

import tf2_ros


class State:
    NO_RAMP = 0
    TO_RAMP = 1
    ON_RAMP = 2


class RampNavigateNode(Node):
    def __init__(self):
        super().__init__('ramp_navigate')

        # State machine
        self.state = State.NO_RAMP
        self.ramps_to_cross = 1
        self.pre_ramp_detections = 0
        self.no_ramp_period = 0

        # Moving average state
        self.xmid = 0.0
        self.ymid = 0.0
        self.px = 0.0
        self.py = 0.0
        self.ramp2map = np.eye(2)

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Subscriber for ramp segment detections
        self.ramp_seg_sub = self.create_subscription(
            PoseArray,
            '/ramp_seg',
            self.ramp_front_callback,
            10
        )

        # Publishers
        self.ramp_naving_pub = self.create_publisher(Bool, '/ramp_naving', 5)
        self.ramp_routine_pub = self.create_publisher(Bool, '/ramp_routine', 5)

        self.get_logger().info('RampNavigateNode started, listening to /ramp_seg')

    def pass_length(self, poses) -> bool:
        """Check if the detected ramp segment is within expected length range."""
        if len(poses) < 2:
            return False

        front = poses[0].position
        back = poses[-1].position

        dx = front.x - back.x
        dy = front.y - back.y
        dz = front.z - back.z

        incline_len2 = dx * dx + dy * dy + dz * dz

        min_len = 2.5
        max_len = 4.0

        return min_len * min_len <= incline_len2 <= max_len * max_len

    def get_robot_position(self):
        """Get robot position in map frame via TF2."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time()
            )
            return (
                transform.transform.translation.x,
                transform.transform.translation.y
            )
        except Exception as ex:
            self.get_logger().warn(f'Could not get transform: {ex}')
            return None

    def send_goal(self, x: float, y: float):
        """Send a navigation goal to Nav2."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            return None

        return self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback_msg):
        """Handle feedback from Nav2 action."""
        pass  # Can log progress if needed

    def cross_ramp(self):
        """Navigate across the ramp with incremental goals."""
        self.get_logger().info('ON RAMP: Initiating ramp crossing')

        # Publish that we're on the ramp
        is_on_ramp = Bool()
        is_on_ramp.data = True
        self.ramp_routine_pub.publish(is_on_ramp)

        ramp_traverse_dist = 8.0  # Total distance to traverse
        traverse_count = 8  # Number of waypoints across ramp

        # Increment vector in map frame
        incr = self.ramp2map @ np.array([ramp_traverse_dist / traverse_count, 0.0])

        px = self.xmid
        py = self.ymid

        # Send incremental goals across ramp
        for i in range(traverse_count):
            px += incr[0]
            py += incr[1]

            self.get_logger().info(f'Ramp crossing goal {i+1}/{traverse_count}: ({px:.2f}, {py:.2f})')

            future = self.send_goal(px, py)
            if future:
                # Wait for this goal to complete before sending next
                rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
                goal_handle = future.result()
                if goal_handle and goal_handle.accepted:
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)

        self.get_logger().info('Finished Ramp Crossing')

        # Reset state
        self.state = State.NO_RAMP
        self.ramps_to_cross -= 1

        # Notify waypoint navigation to resume
        naving_msg = Bool()
        naving_msg.data = False
        self.ramp_naving_pub.publish(naving_msg)

        is_on_ramp.data = False
        self.ramp_routine_pub.publish(is_on_ramp)

    def ramp_front_callback(self, ramp_seg: PoseArray):
        """Process ramp segment detections."""
        # Skip if already on ramp or no more ramps to cross
        # if self.state == State.ON_RAMP or self.ramps_to_cross <= 0:
        #     self.get_logger().info('EARLY OUT BC ON RAMP ALREADY')
        #     return

        # # Validate ramp segment length
        # if not self.pass_length(ramp_seg.poses):
        #     if self.pre_ramp_detections > 0:
        #         self.no_ramp_period += 1
        #         if self.no_ramp_period > 3:
        #             self.pre_ramp_detections = 0
        #             self.no_ramp_period = 0
        #     return

        # self.pre_ramp_detections += 1

        # # Get robot position
        # robot_pos = self.get_robot_position()
        # if robot_pos is None:
        #     return

        # # Calculate ramp midpoint
        # front = ramp_seg.poses[0].position
        # back = ramp_seg.poses[-1].position

        # x_len = back.x - front.x
        # y_len = back.y - front.y
        # length = math.sqrt(x_len * x_len + y_len * y_len)

        # if length < 0.01:
        #     return

        # # Rotation matrix from ramp frame to map frame
        # ramp2map_new = np.array([
        #     [y_len, x_len],
        #     [-x_len, y_len]
        # ]) / length

        # # Calculate midpoint in front of ramp
        # mid = np.array([-1.0, 0.5 * length])
        # midmap = self.ramp2map @ mid + np.array([front.x, front.y])

        # # If goal is too far, use closer point
        # goal_dist2 = (midmap[0] - robot_pos[0])**2 + (midmap[1] - robot_pos[1])**2
        # if goal_dist2 > 9.0:  # 3.0^2
        #     midmap = self.ramp2map @ np.array([0.0, 0.5 * length]) + np.array([front.x, front.y])

        # # Moving average smoothing
        # mvavg = 0.5
        # mvavg_st = 1.0 - mvavg

        # self.ramp2map = self.ramp2map * mvavg_st + mvavg * ramp2map_new
        # self.xmid = self.xmid * mvavg_st + mvavg * midmap[0]
        # self.ymid = self.ymid * mvavg_st + mvavg * midmap[1]

        # self.px = self.xmid
        # self.py = self.ymid

        # # State transitions
        # if self.state == State.NO_RAMP:
        #     if self.pre_ramp_detections < 10:
        #         return
        #     else:
        #         self.state = State.TO_RAMP
        #         self.get_logger().info('STATE CHANGE: TO RAMP')
        #         self.pre_ramp_detections = 0

        #         # Notify waypoint navigation to pause
        #         naving_msg = Bool()
        #         naving_msg.data = True
        #         self.ramp_naving_pub.publish(naving_msg)
        #         self.get_logger().info('Ramp detected! Taking over navigation.')

        # # Send goal to ramp entrance
        # self.send_goal(self.px, self.py)

        # # Check if close enough to start crossing
        # goal_error2 = (self.px - robot_pos[0])**2 + (self.py - robot_pos[1])**2
        # if goal_error2 < 2.0:
        #     self.get_logger().info('STATE CHANGE: ON RAMP')
        #     self.state = State.ON_RAMP
            # self.cross_ramp()


def main(args=None):
    rclpy.init(args=args)
    node = RampNavigateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
