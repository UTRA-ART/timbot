#!/usr/bin/env python3
"""
simple_nav_node: open-loop sequential goal follower.

Takes a list of relative goals [dx, dy, dyaw] (each goal expressed in the
rover's local frame at the moment that goal becomes active) and drives the
rover through them by publishing geometry_msgs/Twist on /cmd_vel.

Assumes no obstacles. Uses /odometry/local for pose feedback.

Per-goal control sequence (turn -> drive -> align):
  1. Rotate in place to face the (dx, dy) point.
  2. Drive forward until within position_tolerance of the point.
  3. Rotate in place to the target yaw (start_yaw + dyaw).
"""

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class SimpleNavNode(Node):
    # control phases
    PHASE_TURN_TO_POINT = 0
    PHASE_DRIVE = 1
    PHASE_ALIGN_YAW = 2
    PHASE_DONE = 3

    def __init__(self):
        super().__init__("simple_nav_node")

        # Goals as a JSON-encoded list of [x, y, yaw] triplets. Each triplet is
        # interpreted in the rover's local frame at the moment that goal becomes
        # active (i.e. each goal is relative to where the previous one ended).
        # JSON is used because ROS 2 parameter types don't allow nested arrays.
        self.declare_parameter("goals", "[[2.0, 0.0, 0.0]]")

        self.declare_parameter("linear_speed", 0.5)         # m/s
        self.declare_parameter("angular_speed", 0.6)        # rad/s
        self.declare_parameter("position_tolerance", 0.10)  # m
        self.declare_parameter("angular_tolerance", 0.05)   # rad
        self.declare_parameter("control_rate", 20.0)        # Hz
        # If the (dx, dy) vector is shorter than this, skip the turn+drive phases
        # and only do the final yaw alignment.
        self.declare_parameter("min_drive_distance", 0.05)  # m
        # Proportional gains. Output is clamped to the speed limits above.
        self.declare_parameter("kp_linear", 0.8)
        self.declare_parameter("kp_angular", 1.5)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.angular_tolerance = float(self.get_parameter("angular_tolerance").value)
        self.min_drive_distance = float(self.get_parameter("min_drive_distance").value)
        self.kp_linear = float(self.get_parameter("kp_linear").value)
        self.kp_angular = float(self.get_parameter("kp_angular").value)

        self.goals = self._parse_goals(self.get_parameter("goals").value)
        self.get_logger().info(f"Loaded {len(self.goals)} relative goal(s).")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.done_pub = self.create_publisher(Bool, "~/done", 1)
        self.create_subscription(Odometry, "/odometry/local", self._odom_cb, 10)

        self.current_pose = None  # (x, y, yaw)

        self.goal_idx = 0
        self.phase = self.PHASE_TURN_TO_POINT
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self._goal_initialized = False

        rate = float(self.get_parameter("control_rate").value)
        self.create_timer(1.0 / rate, self._control_step)

    def _parse_goals(self, raw):
        try:
            parsed = json.loads(raw)
        except (TypeError, json.JSONDecodeError) as e:
            self.get_logger().error(
                f"Could not parse 'goals' as JSON ({e}). Expected a list of [x, y, yaw] "
                "triplets, e.g. '[[2.0, 0.0, 0.0], [0.0, 0.0, 1.5708]]'. No goals will be executed."
            )
            return []
        if not isinstance(parsed, list) or not all(
            isinstance(g, (list, tuple)) and len(g) == 3 for g in parsed
        ):
            self.get_logger().error(
                "'goals' must be a list of [x, y, yaw] triplets. No goals will be executed."
            )
            return []
        return [(float(g[0]), float(g[1]), float(g[2])) for g in parsed]

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.current_pose = (p.x, p.y, yaw)

    def _start_goal(self):
        """Capture the current pose and resolve the next relative goal into world coords."""
        if self.current_pose is None:
            return False
        x, y, yaw = self.current_pose
        dx_local, dy_local, dyaw = self.goals[self.goal_idx]
        # Rotate the local offset by the current yaw to get a world-frame offset.
        c, s = math.cos(yaw), math.sin(yaw)
        self.target_x = x + c * dx_local - s * dy_local
        self.target_y = y + s * dx_local + c * dy_local
        self.target_yaw = wrap_angle(yaw + dyaw)
        self.phase = self.PHASE_TURN_TO_POINT
        self._goal_initialized = True
        self.get_logger().info(
            f"Starting goal {self.goal_idx + 1}/{len(self.goals)}: "
            f"local=({dx_local:.2f}, {dy_local:.2f}, {math.degrees(dyaw):.1f} deg) -> "
            f"world target=({self.target_x:.2f}, {self.target_y:.2f}, "
            f"{math.degrees(self.target_yaw):.1f} deg)"
        )
        return True

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_done(self):
        msg = Bool()
        msg.data = True
        self.done_pub.publish(msg)

    def _control_step(self):
        if not self.goals:
            return
        if self.current_pose is None:
            return  # wait for first odom

        if self.goal_idx >= len(self.goals):
            if self.phase != self.PHASE_DONE:
                self.get_logger().info("All goals reached.")
                self._stop()
                self._publish_done()
                self.phase = self.PHASE_DONE
            return

        if not self._goal_initialized:
            if not self._start_goal():
                return

        x, y, yaw = self.current_pose
        cmd = Twist()

        if self.phase == self.PHASE_TURN_TO_POINT:
            dx = self.target_x - x
            dy = self.target_y - y
            dist = math.hypot(dx, dy)
            if dist < self.min_drive_distance:
                # Already at the point; skip directly to final yaw alignment.
                self.phase = self.PHASE_ALIGN_YAW
            else:
                desired_heading = math.atan2(dy, dx)
                yaw_err = wrap_angle(desired_heading - yaw)
                if abs(yaw_err) < self.angular_tolerance:
                    self.phase = self.PHASE_DRIVE
                else:
                    cmd.angular.z = float(
                        np.clip(
                            self.kp_angular * yaw_err,
                            -self.angular_speed,
                            self.angular_speed,
                        )
                    )

        elif self.phase == self.PHASE_DRIVE:
            dx = self.target_x - x
            dy = self.target_y - y
            dist = math.hypot(dx, dy)
            if dist < self.position_tolerance:
                self.phase = self.PHASE_ALIGN_YAW
            else:
                desired_heading = math.atan2(dy, dx)
                yaw_err = wrap_angle(desired_heading - yaw)
                # If we've drifted too far off heading, stop and re-aim.
                if abs(yaw_err) > 3.0 * self.angular_tolerance:
                    self.phase = self.PHASE_TURN_TO_POINT
                else:
                    cmd.linear.x = float(
                        np.clip(self.kp_linear * dist, 0.0, self.linear_speed)
                    )
                    cmd.angular.z = float(
                        np.clip(
                            self.kp_angular * yaw_err,
                            -self.angular_speed,
                            self.angular_speed,
                        )
                    )

        elif self.phase == self.PHASE_ALIGN_YAW:
            yaw_err = wrap_angle(self.target_yaw - yaw)
            if abs(yaw_err) < self.angular_tolerance:
                self.get_logger().info(f"Reached goal {self.goal_idx + 1}.")
                self._stop()
                self.goal_idx += 1
                self._goal_initialized = False
                return
            cmd.angular.z = float(
                np.clip(
                    self.kp_angular * yaw_err,
                    -self.angular_speed,
                    self.angular_speed,
                )
            )

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
