#!/usr/bin/env python3
"""
Relay node: converts PoseStamped → PoseWithCovarianceStamped.

Cartographer publishes /tracked_pose as PoseStamped, but robot_localization's
EKF node requires PoseWithCovarianceStamped for pose inputs. This node bridges
that gap by copying the pose and adding a configurable covariance matrix.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class NavRelay(Node):
    def __init__(self):
        super().__init__('nav_relay')

        # Configurable covariance diagonal values (position xyz, orientation rpy)
        # ~0.22m std dev

        self.declare_parameter('nav_input_topic', '/nav_vel')
        self.declare_parameter('teleop_input_topic', '/teleop_vel')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('pause_topic', '/pause_navigation')


     
        nav_input_topic = self.get_parameter('nav_input_topic').value
        teleop_input_topic = self.get_parameter('teleop_input_topic').value
        output_topic = self.get_parameter('output_topic').value

       
        self.vel_pub = self.create_publisher(Twist, output_topic, 10)
        self.nav_input_topic = self.create_subscription(Twist, nav_input_topic, self.nav_callback, 20)
        self.teleop_vel_sub = self.create_subscription(Twist, teleop_input_topic, self.teleop_callback, 1)
        self.mode_sub = self.create_subscription(Bool, 'pause_topic', self.pause_callback, 1)
        self.is_paused = False # we default to being in autonomy mode

        self.get_logger().info(
            f'Relaying {teleop_input_topic} and {nav_input_topic} → {output_topic} '
        )

        self.nav_vel = None
        self.key_vel = None

    def pause_callback(self, msg: Bool):
        self.is_paused = msg.data

    def nav_callback(self, msg: Twist):
        if not self.is_paused:
            self.pub.publish(msg)
    
    def teleop_callback(self, msg: Twist):
        if self.is_paused:
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavRelay()
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
