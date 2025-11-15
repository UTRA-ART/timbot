#!/usr/bin/env python

import rclpy
from nav_msgs.msg import Odometry

class ZeroOdomPublisherNode(rclpy.Node):
    def __init__(self):
        super().__init__('zero_odom')

        # Makes publisher
        self.publisher = self.create_publisher(Odometry, 'zero_odom', 1)

        # Time intervals
        timer_period = 1/5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Message to send
        self.zero_odom = Odometry()
        self.zero_odom.header.frame_id = "map"
        self.zero_odom.child_frame_id = "base_link"

    def timer_callback(self):
        self.publisher.publish(self.zero_odom)


def main(args = None):
    rclpy.init(args=args)

    odom_plotter_node = ZeroOdomPublisherNode()
    rclpy.spin(odom_plotter_node)
    
    odom_plotter_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()