#!/usr/bin/env python3
"""
BNO085 IMU driver node.

Reads orientation + IMU data from the BNO085 over serial (ENU frame,
already corrected by firmware) and publishes:
  - /imu/data        (sensor_msgs/Imu)             — primary feed for EKF/navsat
  - imu_enu          (geometry_msgs/Vector3Stamped) — roll/pitch/yaw in degrees

Serial packet: 10 comma-separated floats — ax, ay, az, gx, gy, gz, qw, qx, qy, qz
"""

import math
import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu


def _quat_to_euler_deg(qw, qx, qy, qz):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def _parse_packet(line: str):
    """Return list of 10 floats or None if invalid."""
    parts = line.split(',')
    if len(parts) != 10:
        return None
    try:
        vals = list(map(float, parts))
    except ValueError:
        return None
    if any(math.isnan(v) for v in vals):
        return None
    return vals


class BNO085ImuNode(Node):

    def __init__(self):
        super().__init__('bno085_imu')

        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('orientation_stddev', 0.01)
        self.declare_parameter('yaw_offset', 0.0)  # degrees, same convention as imu_relay

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        orientation_stddev = float(self.get_parameter('orientation_stddev').value)
        self._orientation_cov = orientation_stddev ** 2
        self._yaw_offset_rad = math.radians(float(self.get_parameter('yaw_offset').value))

        self.ser = serial.Serial(port, baud, timeout=1)
        self.ser.reset_input_buffer()  # flush stale bytes from before we opened

        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.enu_pub = self.create_publisher(Vector3Stamped, 'imu_enu', 10)

        self._no_data_count = 0

        self.create_timer(1.0 / 50.0, self._read_serial)

        self.get_logger().info(f'BNO085 IMU running on {port} @ {baud} baud')

    def _read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()

            if not line:
                self._no_data_count += 1
                if self._no_data_count % 100 == 0:
                    self.get_logger().warn(
                        f'BNO085: no data received for {self._no_data_count} cycles '
                        f'(check Arduino is running and sending on {self.ser.port})'
                    )
                return

            self._no_data_count = 0

            vals = _parse_packet(line)
            if vals is None:
                self.get_logger().debug(f'BNO085: skipped malformed packet: {line!r}')
                return

            ax, ay, az, gx, gy, gz, qw, qx, qy, qz = vals

            # Apply yaw offset: rotate quaternion about Z by yaw_offset_rad
            if self._yaw_offset_rad != 0.0:
                half = self._yaw_offset_rad / 2.0
                ow = math.cos(half)
                oz = math.sin(half)
                qw, qx, qy, qz = (
                    ow * qw - oz * qz,
                    ow * qx - oz * qy,
                    ow * qy + oz * qx,
                    ow * qz + oz * qw,
                )

            now = self.get_clock().now().to_msg()
            cov = self._orientation_cov

            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = 'imu_link'
            imu.orientation.w = qw
            imu.orientation.x = qx
            imu.orientation.y = qy
            imu.orientation.z = qz
            imu.orientation_covariance = [cov, 0.0, 0.0, 0.0, cov, 0.0, 0.0, 0.0, cov]
            imu.angular_velocity.x = gx
            imu.angular_velocity.y = gy
            imu.angular_velocity.z = gz
            imu.angular_velocity_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02]
            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az
            imu.linear_acceleration_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02]
            self.imu_pub.publish(imu)

            roll_d, pitch_d, yaw_d = _quat_to_euler_deg(qw, qx, qy, qz)
            enu = Vector3Stamped()
            enu.header.stamp = now
            enu.header.frame_id = 'imu_link'
            enu.vector.x = roll_d
            enu.vector.y = pitch_d
            enu.vector.z = yaw_d
            self.enu_pub.publish(enu)

        except Exception as e:
            self.get_logger().warn(f'BNO085 read error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BNO085ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if node.ser.is_open:
            node.ser.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
