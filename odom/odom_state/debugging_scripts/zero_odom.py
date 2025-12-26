#!/usr/bin/env python3
"""
Zero Odometry Publisher (Testing/Reset Tool)

Purpose:
    Publishes an odometry message with all zeros (position, velocity, orientation).
    Used for testing odometry consumers, providing a known baseline, or as a 
    fallback/reset mechanism during development.

Publishes to:
    - zero_odom (nav_msgs/Odometry): All-zero odometry at 5 Hz

Usage:
    ros2 run odom_state zero_odom
    
Note:
    Not used in normal operation - purely a debugging/testing utility.
"""

import rclpy
from nav_msgs.msg import Odometry

class ZeroOdomPublisherNode(rclpy.Node):
    def __init__(self):
        super().__init__('zero_odom')

        # Create publisher for zero odometry
        self.publisher = self.create_publisher(Odometry, 'zero_odom', 1)

        # Publish at 5 Hz
        timer_period = 1/5  # seconds (0.2s = 5 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create odometry message with all zeros (default values)
        # All pose and twist values remain at 0.0
        self.zero_odom = Odometry()
        self.zero_odom.header.frame_id = "map"
        self.zero_odom.child_frame_id = "base_link"

    def timer_callback(self):
        """Publish zero odometry message at regular intervals"""
        self.publisher.publish(self.zero_odom)


def main(args = None):
    rclpy.init(args=args)

    node = ZeroOdomPublisherNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()