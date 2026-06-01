#!/usr/bin/env python3

import math
import serial

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray


def quat_to_euler(w, x, y, z):
    # Roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class IMUSerialNode(Node):

    def __init__(self):
        super().__init__('imu_serial_node')

        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.ser = serial.Serial(port, baud, timeout=1)

        self.imu_pub = self.create_publisher(
            Imu,
            '/BNO085/imu_data',
            10
        )

        self.euler_pub = self.create_publisher(
            Float32MultiArray,
            '/BNO085/euler',
            10
        )

        self.get_logger().info(
            f'BNO085 serial node running on {port} @ {baud}'
        )

        # 50 Hz timer
        self.timer = self.create_timer(
            1.0 / 50.0,
            self.read_serial
        )

    def safe_float_parse(self, parts):
        try:
            vals = list(map(float, parts))

            if len(vals) != 10:
                return None

            if any(math.isnan(v) for v in vals):
                return None

            return vals

        except Exception:
            return None

    def read_serial(self):
        try:
            line = self.ser.readline().decode(
                'utf-8',
                errors='ignore'
            ).strip()

            if not line:
                return

            parts = line.split(',')

            if len(parts) != 10:
                return

            parsed = self.safe_float_parse(parts)

            if parsed is None:
                return

            ax, ay, az, gx, gy, gz, qw, qx, qy, qz = parsed

            #
            # IMU MESSAGE
            #
            imu_msg = Imu()

            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            imu_msg.orientation.w = qw
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz

            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz

            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az

            imu_msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]

            imu_msg.angular_velocity_covariance = [
                0.02, 0.0, 0.0,
                0.0, 0.02, 0.0,
                0.0, 0.0, 0.02
            ]

            imu_msg.linear_acceleration_covariance = [
                0.02, 0.0, 0.0,
                0.0, 0.02, 0.0,
                0.0, 0.0, 0.02
            ]

            self.imu_pub.publish(imu_msg)

            #
            # EULER MESSAGE (DEGREES)
            #
            roll, pitch, yaw = quat_to_euler(
                qw, qx, qy, qz
            )

            euler_msg = Float32MultiArray()
            euler_msg.data = [
                math.degrees(roll),
                math.degrees(pitch),
                math.degrees(yaw)
            ]

            self.euler_pub.publish(euler_msg)

        except Exception as e:
            self.get_logger().warn(
                f'IMU node error: {e}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = IMUSerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if node.ser.is_open:
        node.ser.close()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
