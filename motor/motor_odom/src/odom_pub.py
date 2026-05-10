#!/usr/bin/env python3
"""
Wheel Odometry Publisher
========================
Computes odometry from wheel tick counts and publishes as nav_msgs/Odometry.

Subscribed to:
  /right_wheel/ticks  (std_msgs/Int32)
  /left_wheel/ticks   (std_msgs/Int32)
  /right_wheel/direction (std_msgs/Bool)
  /left_wheel/direction  (std_msgs/Bool)
  rover_pose/set   (geometry_msgs/PoseStamped)
  rover_pose/reset (std_msgs/Bool)

Publishes:
  odom (nav_msgs/Odometry)

History:
  2023-04-29  Initial version (C++)
  2024-04-10  Direction fix
  2024-05-20  Subscribe to wheel/ticks
  2024-06-01  Pose reset support
  2026-02-28  ROS2 Port (C++)
  2026-04-20  Python port
"""

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Bool, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Convert a yaw angle (radians) to a Quaternion message."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class WheelOdomPub(Node):

    # --- Constants ---
    INITIAL_X = 0.0
    INITIAL_Y = 0.0
    INITIAL_THETA = 1e-11  # tiny offset to avoid division by zero

    PI = math.pi
    WHEEL_RADIUS = 0.1375          # metres (~9.8 in / 24.9 cm diameter)
    CIRCUMFERENCE = 2.0 * PI * WHEEL_RADIUS
    WHEEL_BASE = 0.69             # centre-to-centre of left/right tyres

    A = 0.3114                    # slope of rpm / (ticks per second)
    TICKS_PER_METRE = 1.0 / A / CIRCUMFERENCE * 60.0

    def __init__(self):
        super().__init__('wheel_odom_pub')

        # --- State ---
        self._ticks_left = 0.0
        self._ticks_right = 0.0
        self._distance_left = 0.0
        self._distance_right = 0.0
        self._l_direction = 0
        self._r_direction = 0

        # Current / previous odometry (store only the scalars we need)
        self._x = self.INITIAL_X
        self._y = self.INITIAL_Y
        self._theta = self.INITIAL_THETA
        self._prev_x = self.INITIAL_X
        self._prev_y = self.INITIAL_Y
        self._prev_theta = self.INITIAL_THETA
        self._prev_stamp = self.get_clock().now()

        self._vx = 0.0
        self._wz = 0.0

        # --- Publishers ---
        self._odom_pub = self.create_publisher(Odometry, 'odom', 100)
        self._debug_pub = self.create_publisher(String, 'debug_wheel_odom', 100)

        # --- Subscribers ---
        self.create_subscription(Int32, '/right_wheel/ticks', self._right_ticks_cb, 100)
        self.create_subscription(Int32, '/left_wheel/ticks', self._left_ticks_cb, 100)
        self.create_subscription(Bool, '/right_wheel/direction', self._right_dir_cb, 100)
        self.create_subscription(Bool, '/left_wheel/direction', self._left_dir_cb, 100)
        self.create_subscription(PoseStamped, 'rover_pose/set', self._set_pose_cb, 1)
        self.create_subscription(Bool, 'rover_pose/reset', self._reset_pose_cb, 1)

        # --- 30 Hz timer ---
        self.create_timer(1.0 / 30.0, self._timer_cb)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _right_ticks_cb(self, msg: Int32):
        self._ticks_right = float(msg.data)
        self._distance_right = self._ticks_right / self.TICKS_PER_METRE

    def _left_ticks_cb(self, msg: Int32):
        self._ticks_left = float(msg.data)
        self._distance_left = self._ticks_left / self.TICKS_PER_METRE

    def _right_dir_cb(self, msg: Bool):
        self._r_direction = 1 if msg.data else -1

    def _left_dir_cb(self, msg: Bool):
        self._l_direction = 1 if msg.data else -1

    def _set_pose_cb(self, msg: PoseStamped):
        self._prev_x = msg.pose.position.x
        self._prev_y = msg.pose.position.y
        self._prev_theta = msg.pose.orientation.z

    def _reset_pose_cb(self, msg: Bool):
        if msg.data:
            self._prev_x = 0.0
            self._prev_y = 0.0
            self._prev_theta = 0.0

    # ------------------------------------------------------------------
    # Timer
    # ------------------------------------------------------------------

    def _timer_cb(self):
        self._update_odom()
        self._publish_odom()
        self.get_logger().debug(
            f'left_dir: {self._l_direction}\tright_dir: {self._r_direction}')

    # ------------------------------------------------------------------
    # Odometry math
    # ------------------------------------------------------------------

    def _update_odom(self):
        # Average distance since last cycle
        cycle_distance = (
            (self._r_direction * self._distance_right)
            + (self._l_direction * self._distance_left)
        ) / 2.0

        # Radians turned since last cycle
        diff = (
            (self._r_direction * self._distance_right)
            - (self._l_direction * self._distance_left)
        )
        cycle_angle = math.asin(max(-1.0, min(1.0, diff / self.WHEEL_BASE)))

        # Average angle during the cycle
        avg_angle = cycle_angle / 2.0 + self._prev_theta
        if avg_angle > self.PI:
            avg_angle -= 2.0 * self.PI
        elif avg_angle < -self.PI:
            avg_angle += 2.0 * self.PI

        # New pose
        new_x = self._prev_x + math.cos(avg_angle) * cycle_distance
        new_y = self._prev_y + math.sin(avg_angle) * cycle_distance
        new_theta = cycle_angle + self._prev_theta

        # Guard against NaN from a bad cycle
        if math.isnan(new_x) or math.isnan(new_y) or math.isnan(new_theta):
            new_x = self._prev_x
            new_y = self._prev_y
            new_theta = self._prev_theta

        # Wrap theta to [-pi, pi]
        if new_theta > self.PI:
            new_theta -= 2.0 * self.PI
        elif new_theta < -self.PI:
            new_theta += 2.0 * self.PI

        # Velocity
        now = self.get_clock().now()
        dt = (now - self._prev_stamp).nanoseconds * 1e-9
        if dt > 0.0:
            self._vx = cycle_distance / dt
            self._wz = cycle_angle / dt

        # Store for next cycle
        self._x = new_x
        self._y = new_y
        self._theta = new_theta
        self._prev_x = new_x
        self._prev_y = new_y
        self._prev_theta = new_theta
        self._prev_stamp = now

    def _publish_odom(self):
        q = quaternion_from_yaw(self._theta)

        odom = Odometry()
        odom.header.stamp = self._prev_stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.angular.z = self._wz

        # Covariance: diagonal entries only
        cov = [0.0] * 36
        cov[0] = 0.01    # x
        cov[7] = 0.01    # y
        cov[14] = 0.01   # z
        cov[21] = 0.1    # roll
        cov[28] = 0.1    # pitch
        cov[35] = 0.1    # yaw
        odom.pose.covariance = cov

        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
