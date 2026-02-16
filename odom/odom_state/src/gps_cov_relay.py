#!/usr/bin/env python3
"""
GPS Covariance Relay Node

Subscribes to /gps/fix_raw, adds position covariance (since Ignition's NavSat 
bridge doesn't populate it), and republishes to /gps/fix.

This is needed because robot_localization's navsat_transform_node passes through
the covariance to /odometry/gps. Zero covariance = EKF trusts GPS infinitely = jitter.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GpsCovRelay(Node):
    def __init__(self):
        super().__init__('gps_cov_relay')

        # Configurable covariance (diagonal, meters^2)
        self.declare_parameter('horizontal_stddev', 2.0)
        self.declare_parameter('vertical_stddev', 3.0)

        h_std = self.get_parameter('horizontal_stddev').value
        v_std = self.get_parameter('vertical_stddev').value

        self.h_var = h_std ** 2  # 4.0 m^2
        self.v_var = v_std ** 2  # 9.0 m^2

        self.sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.callback, 10)
        self.pub = self.create_publisher(NavSatFix, '/gps/fix_cov', 10)

        self.get_logger().info(
            f'GPS covariance relay: h_var={self.h_var:.1f}, v_var={self.v_var:.1f}')

    def callback(self, msg: NavSatFix):
        # Set diagonal covariance [lat, lon, alt] in row-major 3x3
        msg.position_covariance = [
            self.h_var, 0.0, 0.0,
            0.0, self.h_var, 0.0,
            0.0, 0.0, self.v_var
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        # Ensure status is set to FIX
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            msg.status.status = NavSatStatus.STATUS_FIX

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GpsCovRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
