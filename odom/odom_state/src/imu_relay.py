#!/usr/bin/env python3
"""
IMU Relay Node (NED -> ENU)

Subscribes to an IMU topic that reports NED data and republishes it as ENU.
The outgoing frame_id remains imu_link to preserve downstream expectations.
"""

import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu


class ImuRelay(Node):
    def __init__(self):
        super().__init__('imu_relay')

        self.declare_parameter('input_topic', 'imu/data_raw')
        self.declare_parameter('output_topic', 'imu/data_processed')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('euler_topic', 'imu_euler')
        self.declare_parameter('euler_untrans_topic', 'imu_untrans_euler')
        self.declare_parameter('yaw_offset_deg', -90.0)

    
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        euler_topic = self.get_parameter('euler_topic').value
        euler_untrans_topic = self.get_parameter('euler_untrans_topic').value
        yaw_offset_deg = float(self.get_parameter('yaw_offset_deg').value)

        self.sub = self.create_subscription(Imu, input_topic, self.callback, 10)
        self.pub = self.create_publisher(Imu, output_topic, 10)
        self.euler_pub = self.create_publisher(Vector3Stamped, euler_topic, 10)
        self.euler_untrans_pub = self.create_publisher(Vector3Stamped, euler_untrans_topic, 10)

        self.get_logger().info(
            f'IMU relay: {input_topic} (NED) -> {output_topic} (ENU), frame_id={self.frame_id}')
        self.get_logger().info(
            f'IMU Euler topics: transformed={euler_topic}, untransformed={euler_untrans_topic}')
        self.get_logger().info(
            f'IMU yaw offset (NED +Z): {yaw_offset_deg:.1f} deg')

        inv_sqrt2 = 1.0 / math.sqrt(2.0)
        self.q_ned_to_enu = (inv_sqrt2, inv_sqrt2, 0.0, 0.0)
        self.r_ned_to_enu = (
            (0.0, 1.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 0.0, -1.0),
        )

        self.q_yaw_offset = self.make_yaw_offset_quat(yaw_offset_deg)

        self.add_on_set_parameters_callback(self.on_param_set)

    def callback(self, msg: Imu):
        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.frame_id

        corrected_orientation = self.apply_yaw_offset(msg.orientation)

        out.orientation = self.rotate_quaternion(corrected_orientation)
        out.angular_velocity = self.rotate_vector(msg.angular_velocity)
        out.linear_acceleration = self.rotate_vector(msg.linear_acceleration)

        out.orientation_covariance = self.rotate_covariance(msg.orientation_covariance)
        out.angular_velocity_covariance = self.rotate_covariance(msg.angular_velocity_covariance)
        out.linear_acceleration_covariance = self.rotate_covariance(msg.linear_acceleration_covariance)

        self.pub.publish(out)

        self.publish_euler(msg, corrected_orientation, out)

    def publish_euler(self, msg_in: Imu, corrected_orientation, msg_out: Imu):
        euler_untrans = Vector3Stamped()
        euler_untrans.header = msg_in.header
        euler_untrans.vector = self.quat_to_euler(corrected_orientation)
        self.euler_untrans_pub.publish(euler_untrans)

        euler_trans = Vector3Stamped()
        euler_trans.header = msg_out.header
        euler_trans.vector = self.quat_to_euler(msg_out.orientation)
        self.euler_pub.publish(euler_trans)

    def rotate_vector(self, vec):
        x = self.r_ned_to_enu[0][0] * vec.x + self.r_ned_to_enu[0][1] * vec.y + self.r_ned_to_enu[0][2] * vec.z
        y = self.r_ned_to_enu[1][0] * vec.x + self.r_ned_to_enu[1][1] * vec.y + self.r_ned_to_enu[1][2] * vec.z
        z = self.r_ned_to_enu[2][0] * vec.x + self.r_ned_to_enu[2][1] * vec.y + self.r_ned_to_enu[2][2] * vec.z

        vec_out = type(vec)()
        vec_out.x = x
        vec_out.y = y
        vec_out.z = z
        return vec_out

    def rotate_quaternion(self, quat):
        q_in = (quat.x, quat.y, quat.z, quat.w)
        q_out = self.quat_multiply(self.q_ned_to_enu, q_in)

        quat_out = type(quat)()
        quat_out.x, quat_out.y, quat_out.z, quat_out.w = q_out
        return quat_out

    def apply_yaw_offset(self, quat):
        q_in = (quat.x, quat.y, quat.z, quat.w)
        q_out = self.quat_multiply(self.q_yaw_offset, q_in)

        quat_out = type(quat)()
        quat_out.x, quat_out.y, quat_out.z, quat_out.w = q_out
        return quat_out

    def make_yaw_offset_quat(self, yaw_offset_deg):
        yaw_offset_rad = math.radians(yaw_offset_deg)
        half = yaw_offset_rad * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def on_param_set(self, params):
        updated = False
        for param in params:
            if param.name == 'yaw_offset_deg':
                try:
                    self.q_yaw_offset = self.make_yaw_offset_quat(float(param.value))
                    updated = True
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False, reason='yaw_offset_deg must be a number')

        if updated:
            self.get_logger().info('Updated yaw_offset_deg parameter')

        return SetParametersResult(successful=True)

    def rotate_covariance(self, cov):
        if len(cov) != 9:
            return cov

        r = self.r_ned_to_enu
        c = (
            (cov[0], cov[1], cov[2]),
            (cov[3], cov[4], cov[5]),
            (cov[6], cov[7], cov[8]),
        )

        rc = self.mul3(r, c)
        rct = self.mul3(rc, self.transpose3(r))

        return [
            rct[0][0], rct[0][1], rct[0][2],
            rct[1][0], rct[1][1], rct[1][2],
            rct[2][0], rct[2][1], rct[2][2],
        ]

    def mul3(self, a, b):
        return (
            (
                a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
                a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
                a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
            ),
            (
                a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
                a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
                a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
            ),
            (
                a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
                a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
                a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
            ),
        )

    def transpose3(self, a):
        return (
            (a[0][0], a[1][0], a[2][0]),
            (a[0][1], a[1][1], a[2][1]),
            (a[0][2], a[1][2], a[2][2]),
        )

    def quat_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    def quat_to_euler(self, quat):
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)

        vec = Vector3Stamped().vector
        vec.x = roll
        vec.y = pitch
        vec.z = yaw
        return vec

def main(args=None):
    try:
        rclpy.init(args=args)
        node = ImuRelay()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Shutting down imu_relay node...')
        except Exception as e:
            node.get_logger().error(f'Unexpected error: {e}')
        finally:
            node.destroy_node()
    except Exception as e:
        print(f'Failed to initialize IMU Relay: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
