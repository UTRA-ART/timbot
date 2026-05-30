#!/usr/bin/env python3
"""
IMU NED -> ENU Relay Node
=========================

The Phidgets onboard AHRS (``use_orientation:=True``) reports its orientation
quaternion in a NED-referenced world frame (X=North, Y=East, Z=Down). ROS /
``robot_localization`` / ``navsat_transform`` all assume ENU (REP-103). This
node:

1. Subscribes to the AHRS Imu message (``/imu/data_raw``).
2. Converts ONLY the orientation quaternion from NED-world to ENU-world,
   passing accelerometer / gyro (and their covariances) through unchanged.
3. Republishes the corrected message on ``/imu/data``.
4. Publishes the *absolute* orientation, in degrees (roll, pitch, yaw), on two
   debug topics so the heading can be sanity-checked on a bench:
       - ``imu_ned`` : orientation exactly as received from the AHRS (NED).
       - ``imu_enu`` : orientation after the NED->ENU + yaw_offset correction
                       (i.e. what is published on ``/imu/data``).

Math
----
The orientation ``q_ned`` satisfies ``v_ned = R(q_ned) . v_body``.  We want
``q_enu`` with ``v_enu = R(q_enu) . v_body``.  The fixed NED->ENU world
transform T maps (N,E,D)->(E,N,U); as a quaternion that is a 180 deg rotation
about (1,1,0)/sqrt(2):

    q_T (x,y,z,w) = (sqrt(2)/2, sqrt(2)/2, 0, 0)

so  ``q_enu = q_T (x) q_ned``.

The BODY frame is left as the sensor's own axes (frame_id ``imu_link``). The
``imu_link``->``base_link`` mounting (incl. any Z-down flip) is described by the
URDF imu_joint and applied downstream by TF -- it is NOT this node's job.

``yaw_offset`` (degrees) adds a world-frame yaw rotation about ENU Up. Use it to
calibrate the heading if bench verification shows a constant offset between the
reported heading and true ENU yaw (leave 0 unless you measure such an offset).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped


SQRT2_2 = math.sqrt(2.0) / 2.0
# NED-world -> ENU-world correction (x, y, z, w)
Q_NED_TO_ENU = (SQRT2_2, SQRT2_2, 0.0, 0.0)


def quat_mul(a, b):
    """Hamilton product a (x) b, both (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_to_euler_deg(q):
    """Convert a quaternion (x, y, z, w) to roll/pitch/yaw in DEGREES.

    Uses the standard ZYX (yaw-pitch-roll) Tait-Bryan convention.
    """
    x, y, z, w = q

    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation), clamped to avoid NaN at the poles
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


class ImuNedToEnuRelay(Node):
    def __init__(self):
        super().__init__('imu_relay')

        self.declare_parameter('input_topic', '/imu/data_raw')
        self.declare_parameter('output_topic', '/imu/data')
        # World-frame yaw calibration offset (degrees), applied about ENU Up.
        self.declare_parameter('yaw_offset', 0.0)
        # Orientation covariance (diagonal stddev, rad). The driver does not fill
        # orientation covariance; navsat needs a finite value to use the heading.
        self.declare_parameter('orientation_stddev', 0.05)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.yaw_offset_deg = float(self.get_parameter('yaw_offset').value)
        ori_var = float(self.get_parameter('orientation_stddev').value) ** 2

        # World-frame correction = (yaw offset) (x) (NED->ENU)
        half = math.radians(self.yaw_offset_deg) / 2.0
        q_yaw = (0.0, 0.0, math.sin(half), math.cos(half))  # yaw about ENU Up
        self.q_world = quat_mul(q_yaw, Q_NED_TO_ENU)
        self.ori_cov_diag = ori_var

        # Allow live re-tuning of yaw_offset without relaunching.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.pub = self.create_publisher(Imu, self.output_topic, 10)
        self.ned_pub = self.create_publisher(Vector3Stamped, 'imu_ned', 10)
        self.enu_pub = self.create_publisher(Vector3Stamped, 'imu_enu', 10)
        self.sub = self.create_subscription(
            Imu,
            self.input_topic,
            self.callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'IMU NED->ENU relay: {self.input_topic} -> {self.output_topic} '
            f'(yaw_offset={self.yaw_offset_deg:.2f} deg); '
            f'debug degrees on imu_ned / imu_enu'
        )

    def _on_set_parameters(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'yaw_offset':
                self.yaw_offset_deg = float(p.value)
                half = math.radians(self.yaw_offset_deg) / 2.0
                q_yaw = (0.0, 0.0, math.sin(half), math.cos(half))
                self.q_world = quat_mul(q_yaw, Q_NED_TO_ENU)
                self.get_logger().info(
                    f'yaw_offset updated to {self.yaw_offset_deg:.2f} deg'
                )
            elif p.name == 'orientation_stddev':
                self.ori_cov_diag = float(p.value) ** 2
        return SetParametersResult(successful=True)

    def _publish_euler(self, publisher, header, quat):
        roll, pitch, yaw = quat_to_euler_deg(quat)
        out = Vector3Stamped()
        out.header = header
        out.vector.x = roll
        out.vector.y = pitch
        out.vector.z = yaw
        publisher.publish(out)

    def callback(self, msg):
        q_ned = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        # Guard against an all-zero (invalid) quaternion from the driver.
        if q_ned == (0.0, 0.0, 0.0, 0.0):
            return

        q_enu = quat_mul(self.q_world, q_ned)

        # Debug: absolute orientation in degrees, before and after correction.
        self._publish_euler(self.ned_pub, msg.header, q_ned)
        self._publish_euler(self.enu_pub, msg.header, q_enu)

        # Replace only the orientation; keep accel/gyro + their covariances + header.
        qx, qy, qz, qw = q_enu
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
