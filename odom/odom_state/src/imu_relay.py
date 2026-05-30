#!/usr/bin/env python3
"""
IMU NED → ENU Relay Node

The Phidgets onboard AHRS (use_orientation:=True) reports its orientation
quaternion in a NED-referenced world frame (X=North, Y=East, Z=Down). ROS /
robot_localization / navsat_transform all assume ENU (REP-103). This node
subscribes to the AHRS Imu message, converts ONLY the orientation quaternion
from NED-world to ENU-world, passes accelerometer/gyro through unchanged, and
republishes.

Math
----
The orientation q_ned satisfies  v_ned = R(q_ned) · v_body.  We want q_enu with
v_enu = R(q_enu) · v_body.  The fixed NED→ENU world transform T maps
(N,E,D)->(E,N,U); as a quaternion that is a 180° rotation about (1,1,0)/√2:

    q_T (x,y,z,w) = (√2/2, √2/2, 0, 0)

so  q_enu = q_yaw ⊗ q_T ⊗ q_ned, where q_yaw is the optional fine-tuning yaw
offset (`extra_yaw_offset_deg`) applied in the ENU world frame.

The BODY frame is left as the sensor's own axes (frame_id imu_link). The
imu_link→base_link mounting (incl. any Z-down flip) is described by the URDF
imu_joint and applied downstream by TF — it is NOT this node's job. Likewise,
angular_velocity and linear_acceleration are body-frame quantities and are
passed through unchanged (NED/ENU is a world-frame convention only).

Live tuning
-----------
`extra_yaw_offset_deg` is dynamically reconfigurable — adjust the heading without
relaunching:

    ros2 param set /imu_relay extra_yaw_offset_deg 37.5

Use it only for a CONSTANT heading offset. If the error varies with heading
(right at North, wrong at East, etc.) that's residual magnetometer calibration,
which an offset cannot fix — recalibrate instead.
"""

import math

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


SQRT2_2 = math.sqrt(2.0) / 2.0
# NED-world -> ENU-world correction (x, y, z, w)
Q_NED_TO_ENU = (SQRT2_2, SQRT2_2, 0.0, 0.0)


def quat_mul(a, b):
    """Hamilton product a ⊗ b, both (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


class ImuNedToEnuRelay(Node):
    def __init__(self):
        super().__init__('imu_relay')

        self.declare_parameter('input_topic', '/imu/data_raw')
        self.declare_parameter('output_topic', '/imu/data')
        self.declare_parameter('extra_yaw_offset_deg', 0.0)
        # Orientation covariance (diagonal, rad^2). The driver does not fill
        # orientation covariance; navsat needs a finite value to use the heading.
        self.declare_parameter('orientation_stddev', 0.05)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self._set_yaw_offset(float(self.get_parameter('extra_yaw_offset_deg').value))
        self.ori_cov_diag = float(self.get_parameter('orientation_stddev').value) ** 2

        # Allow live tuning of the yaw offset (and orientation_stddev).
        self.add_on_set_parameters_callback(self._on_params)

        self.pub = self.create_publisher(Imu, self.output_topic, 10)
        self.sub = self.create_subscription(
            Imu, self.input_topic, self.callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f'IMU NED→ENU relay: {self.input_topic} → {self.output_topic} '
            f'(extra_yaw_offset={self.yaw_offset_deg:.1f}°). '
            f'Live-tune with: ros2 param set {self.get_name()} extra_yaw_offset_deg <deg>'
        )

    def _set_yaw_offset(self, deg: float):
        """Recompute the world-frame correction = q_yaw ⊗ q_T."""
        self.yaw_offset_deg = float(deg)
        half = math.radians(self.yaw_offset_deg) / 2.0
        q_yaw = (0.0, 0.0, math.sin(half), math.cos(half))  # yaw about ENU Up
        self.q_world = quat_mul(q_yaw, Q_NED_TO_ENU)

    def _on_params(self, params):
        for p in params:
            if p.name == 'extra_yaw_offset_deg':
                try:
                    self._set_yaw_offset(float(p.value))
                except (TypeError, ValueError) as exc:
                    return SetParametersResult(
                        successful=False, reason=f'invalid extra_yaw_offset_deg: {exc}'
                    )
                self.get_logger().info(
                    f'extra_yaw_offset_deg set to {self.yaw_offset_deg:.1f}°'
                )
            elif p.name == 'orientation_stddev':
                try:
                    self.ori_cov_diag = float(p.value) ** 2
                except (TypeError, ValueError) as exc:
                    return SetParametersResult(
                        successful=False, reason=f'invalid orientation_stddev: {exc}'
                    )
        return SetParametersResult(successful=True)

    def callback(self, msg: Imu):
        q_ned = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        # Guard against an all-zero (invalid) quaternion from the driver.
        if q_ned == (0.0, 0.0, 0.0, 0.0):
            return

        qx, qy, qz, qw = quat_mul(self.q_world, q_ned)

        # Copy the message (keeps accel/gyro + their covariances + header),
        # replace only the orientation.
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.orientation_covariance = [
            self.ori_cov_diag, 0.0, 0.0,
            0.0, self.ori_cov_diag, 0.0,
            0.0, 0.0, self.ori_cov_diag,
        ]
        self.pub.publish(msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ImuNedToEnuRelay()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Shutting down imu_relay node...')
        finally:
            node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
