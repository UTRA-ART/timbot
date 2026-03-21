#!/usr/bin/env python3
"""
Relay node: converts PoseStamped → PoseWithCovarianceStamped.

Cartographer publishes /tracked_pose as PoseStamped, but robot_localization's
EKF node requires PoseWithCovarianceStamped for pose inputs. This node bridges
that gap by copying the pose and adding a configurable covariance matrix.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class PoseRelay(Node):
    def __init__(self):
        super().__init__('pose_relay')

        # Configurable covariance diagonal values (position xyz, orientation rpy)
        self.declare_parameter('position_covariance', 0.05)   # ~0.22m std dev
        self.declare_parameter('orientation_covariance', 0.01) # ~0.1 rad std dev
        self.declare_parameter('input_topic', '/tracked_pose')
        self.declare_parameter('output_topic', '/tracked_pose_cov')

        pos_cov = self.get_parameter('position_covariance').value
        ori_cov = self.get_parameter('orientation_covariance').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # Build 6x6 diagonal covariance (x, y, z, roll, pitch, yaw)
        self.covariance = [0.0] * 36
        self.covariance[0]  = pos_cov   # x
        self.covariance[7]  = pos_cov   # y
        self.covariance[14] = pos_cov   # z
        self.covariance[21] = ori_cov   # roll
        self.covariance[28] = ori_cov   # pitch
        self.covariance[35] = ori_cov   # yaw

        self.pub = self.create_publisher(PoseWithCovarianceStamped, output_topic, 10)
        self.sub = self.create_subscription(PoseStamped, input_topic, self.callback, 10)

        self.get_logger().info(
            f'Relaying {input_topic} → {output_topic} '
            f'(pos_cov={pos_cov}, ori_cov={ori_cov})'
        )

    def callback(self, msg: PoseStamped):
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.pose.pose = msg.pose
        out.pose.covariance = self.covariance
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PoseRelay()
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
