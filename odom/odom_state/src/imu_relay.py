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

so  q_enu = q_T ⊗ q_ned.

The BODY frame is left as the sensor's own axes (frame_id imu_link). The
imu_link→base_link mounting (incl. any Z-down flip) is described by the URDF
imu_joint and applied downstream by TF — it is NOT this node's job.

`extra_yaw_offset_deg` adds a world-frame yaw rotation for fine-tuning if the
device's NED reference differs from textbook NED (leave 0 unless bench
verification shows a constant heading offset).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
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
        yaw_off = math.radians(float(self.get_parameter('extra_yaw_offset_deg').value))
        ori_var = float(self.get_parameter('orientation_stddev').value) ** 2

        # World-frame correction = (optional yaw offset) ⊗ (NED→ENU)
        half = yaw_off / 2.0
        q_yaw = (0.0, 0.0, math.sin(half), math.cos(half))  # yaw about ENU Up
        self.q_world = quat_mul(q_yaw, Q_NED_TO_ENU)
        self.ori_cov_diag = ori_var

        self.pub = self.create_publisher(Imu, self.output_topic, 10)
        self.sub = self.create_subscription(
            Imu,
            self.input_topic,
            self.callback,
            qos_profile_sensor_data,
            raw=True,
        )

        self.get_logger().info(
            f'IMU NED→ENU relay: {self.input_topic} → {self.output_topic} '
            f'(extra_yaw_offset={math.degrees(yaw_off):.1f}°)'
        )

    def callback(self, msg):
        try:
            msg = deserialize_message(msg, Imu)
        except Exception as exc:
            self.get_logger().warn(f'Failed to deserialize IMU message: {exc}')
            return
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
