#!/usr/bin/env python3

import math
from threading import Lock

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class CustomNavController(Node):
    def __init__(self):
        super().__init__("custom_nav_controller")

        self.declare_parameter("odom_topic", "/odometry/global")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("cmd_vel_topic", "/nav_vel")
        self.declare_parameter("controller_frequency", 10.0)
        self.declare_parameter("lookahead_distance", 0.2)
        self.declare_parameter("goal_tolerance", 0.05)
        self.declare_parameter("kP", 10.5)
        self.declare_parameter("kD", 0.1)
        self.declare_parameter("max_linear_velocity", 0.3)
        self.declare_parameter("max_angular_velocity", 1.5)
        self.declare_parameter("max_forwards_linear_acceleration", 1.0)
        self.declare_parameter("max_angular_acceleration", 2.0)

        odom_topic = str(self.get_parameter("odom_topic").value)
        plan_topic = str(self.get_parameter("plan_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        controller_frequency = float(
            self.get_parameter("controller_frequency").value
        )
        self._odom_topic_name = odom_topic
        self._plan_topic_name = plan_topic
        self.lookahead_distance = float(
            self.get_parameter("lookahead_distance").value
        )
        self.goal_tolerance = float(
            self.get_parameter("goal_tolerance").value
        )
        self.kP = float(self.get_parameter("kP").value)
        self.kD = float(self.get_parameter("kD").value)
        self.max_linear_velocity = float(
            self.get_parameter("max_linear_velocity").value
        )
        self.max_angular_velocity = float(
            self.get_parameter("max_angular_velocity").value
        )
        self.max_forwards_linear_acceleration = float(
            self.get_parameter("max_forwards_linear_acceleration").value
        )
        self.max_angular_acceleration = float(
            self.get_parameter("max_angular_acceleration").value
        )
        self.controller_update_frequency = controller_frequency
        self.previous_angle_error = None

        self._lock = Lock()
        self._latest_odom = None
        self._latest_path = None

        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, 10
        )
        self._plan_sub = self.create_subscription(
            Path, plan_topic, self._path_callback, 10
        )
        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._warned_missing_odom = False
        self._warned_missing_path = False

        period = 1.0 / max(controller_frequency, 0.1)
        self._timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(
            "Custom Python nav controller listening on "
            f"{odom_topic} and {plan_topic}, publishing {cmd_vel_topic}"
        )

    def _odom_callback(self, odometry):
        with self._lock:
            self._latest_odom = odometry

    def _path_callback(self, path):
        with self._lock:
            self._latest_path = path

    def compute_velocity(self, odometry: Odometry, path: Path) -> Twist:
        """
        odometry: latest robot odometry
        path: latest nav_msgs/Path

        Return the velocity that should be published to /nav_vel.
        """
        if not path.poses:
            return Twist()

        robot_x = odometry.pose.pose.position.x
        robot_y = odometry.pose.pose.position.y

        goal = path.poses[-1].pose.position
        goal_distance = math.hypot(goal.x - robot_x, goal.y - robot_y)

        if goal_distance <= self.goal_tolerance:
            self.previous_angle_error = None
            return Twist()

        closest_index = min(
            range(len(path.poses)),
            key=lambda index: (
                (path.poses[index].pose.position.x - robot_x) ** 2
                + (path.poses[index].pose.position.y - robot_y) ** 2
            ),
        )

        lookahead_point = path.poses[-1]

        for point in path.poses[closest_index:]:
            distance = ((point.pose.position.x - robot_x)**2 + (point.pose.position.y - robot_y)**2)**0.5
            if distance >= self.lookahead_distance:
                lookahead_point = point
                break

        orientation = odometry.pose.pose.orientation

        sin_yaw = 2 * (orientation.w * orientation.z
                    + orientation.x * orientation.y)

        cos_yaw = 1 - 2 * (orientation.y**2 + orientation.z**2)

        robot_yaw = math.atan2(sin_yaw, cos_yaw)

        dx = lookahead_point.pose.position.x - robot_x
        dy = lookahead_point.pose.position.y - robot_y

        local_x = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
        local_y = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)

        angle_error = math.atan2(local_y, local_x)
        angle_error = math.atan2(
            math.sin(angle_error),
            math.cos(angle_error),
        )

        if abs(angle_error) > math.pi / 2:
            # Target is more than 90 degrees away: rotate in place.

            dt = 1.0 / self.controller_update_frequency

            if self.previous_angle_error is None:
                derivative_error = 0.0
            else:
                error_change = math.atan2(
                    math.sin(angle_error - self.previous_angle_error),
                    math.cos(angle_error - self.previous_angle_error)
                )
                derivative_error = error_change / dt

            self.previous_angle_error = angle_error

            linear_velocity = 0.0
            angular_velocity = max(
                -self.max_angular_velocity,
                min(
                    self.kP * angle_error + self.kD * derivative_error,
                    self.max_angular_velocity
                )
            )

        else:
            # Target is ahead: follow Pure Pursuit
            self.previous_angle_error = None
            L_squared = local_x**2 + local_y**2

            if L_squared < 1e-6:
                return Twist()

            curvature = 2 * local_y / L_squared

            if abs(curvature) > 1e-6:
                linear_velocity = min(
                    self.max_linear_velocity,
                    self.max_angular_velocity / abs(curvature),
                )
            else:
                linear_velocity = self.max_linear_velocity

            angular_velocity = linear_velocity * curvature

        current_linear_velocity = odometry.twist.twist.linear.x
        current_angular_velocity = odometry.twist.twist.angular.z

        dt = 1.0 / self.controller_update_frequency

        max_linear_change = self.max_forwards_linear_acceleration * dt
        max_angular_change = self.max_angular_acceleration * dt

        linear_velocity = max(
            current_linear_velocity - max_linear_change,
            min(linear_velocity, current_linear_velocity + max_linear_change)
        )

        angular_velocity = max(
            current_angular_velocity - max_angular_change,
            min(angular_velocity, current_angular_velocity + max_angular_change)
        )

        if abs(angle_error) > math.pi / 2 and abs(linear_velocity) > 1e-3:
            angular_velocity = 0.0
            self.previous_angle_error = None

        angular_velocity = max(
            -self.max_angular_velocity,
            min(angular_velocity, self.max_angular_velocity)
        )

        # velocity command
        cmd = Twist()

        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity

        return cmd

    def _control_loop(self):
        with self._lock:
            odometry = self._latest_odom
            path = self._latest_path

        if odometry is None or path is None or not path.poses:
            if odometry is None and not self._warned_missing_odom:
                self.get_logger().warn(
                    f"Waiting for odometry on {self._odom_topic_name}"
                )
                self._warned_missing_odom = True
            if (path is None or not path.poses) and not self._warned_missing_path:
                self.get_logger().warn(
                    f"Waiting for a non-empty Nav2 plan on "
                    f"{self._plan_topic_name}"
                )
                self._warned_missing_path = True
            self._cmd_vel_pub.publish(Twist())
            return

        try:
            velocity = self.compute_velocity(odometry, path)
        except Exception as exc:
            self.get_logger().error(f"Controller function failed: {exc}")
            velocity = Twist()

        self._cmd_vel_pub.publish(velocity)


def main(args=None):
    rclpy.init(args=args)
    node = CustomNavController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
