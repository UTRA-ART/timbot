#!/usr/bin/env python3
"""
Nav/teleop cmd_vel relay.

Forwards either /teleop_vel (manual) or /nav_vel (autonomous) to /cmd_vel based
on the /pause_navigation mode published by teleop_twist_keyboard:
  pause_navigation == True  -> teleop mode (forward /teleop_vel)
  pause_navigation == False -> autonomous  (forward /nav_vel)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# Must match the latched publisher in teleop_twist_keyboard.py. This relay is
# typically launched (via load_waypoints) AFTER the operator has already pressed
# 'p' in teleop, so the mode was published before this node existed. A volatile
# subscription would miss it and stay stuck in the default teleop mode, silently
# dropping all /nav_vel — so the rover never moves in autonomous. TRANSIENT_LOCAL
# delivers the last retained /pause_navigation value the moment we subscribe.
PAUSE_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

class NavRelay(Node):
    def __init__(self):
        super().__init__('nav_relay')


        self.declare_parameter('nav_input_topic', '/nav_vel')
        self.declare_parameter('teleop_input_topic', '/teleop_vel')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('pause_topic', '/pause_navigation')


     
        nav_input_topic = self.get_parameter('nav_input_topic').value
        teleop_input_topic = self.get_parameter('teleop_input_topic').value
        output_topic = self.get_parameter('output_topic').value
        pause_topic = self.get_parameter('pause_topic').value

       
        self.vel_pub = self.create_publisher(Twist, output_topic, 10)

        self.nav_input_topic = self.create_subscription(Twist, nav_input_topic, self.nav_callback, 20)
        self.teleop_vel_sub = self.create_subscription(Twist, teleop_input_topic, self.teleop_callback, 1)
        self.mode_sub = self.create_subscription(Bool, pause_topic, self.pause_callback, PAUSE_QOS)
        self.is_paused = True # default to keyboard mode until the latched pause_navigation arrives

        self.get_logger().info(
            f'Relaying {teleop_input_topic} and {nav_input_topic} → {output_topic} '
        )

        self.nav_vel = None
        self.key_vel = None

    def pause_callback(self, msg: Bool):
        self.is_paused = msg.data

    def nav_callback(self, msg: Twist):
        
        if not self.is_paused:
            self.get_logger().info(
                'we are publishing nav vel'
            )
            self.vel_pub.publish(msg)
    
    def teleop_callback(self, msg: Twist):
        if self.is_paused:
            self.get_logger().info(
                'we are publishing teleop vel'
            )
            self.vel_pub.publish(msg)


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
