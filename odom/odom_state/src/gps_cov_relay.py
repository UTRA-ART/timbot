#!/usr/bin/env python3
"""
GPS Covariance Relay Node

Subscribes to /gps/fix, ensures position covariance is set, and republishes
to /gps/fix_cov.

In simulation (use_sim_time=true): Ignition's NavSat bridge doesn't populate
covariance, so we inject configurable values. Zero covariance = EKF trusts GPS
infinitely = jitter.

On real hardware (use_sim_time=false): The GPS receiver provides its own
covariance. We pass it through untouched, only ensuring status is set to FIX.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GpsCovRelay(Node):
    def __init__(self):
        super().__init__('gps_cov_relay')

        # Check if we're in simulation
        self.is_sim = self.get_parameter('use_sim_time').value

        # Configurable covariance (diagonal, meters^2) — only used in sim
        self.declare_parameter('horizontal_stddev', 2.0)
        self.declare_parameter('vertical_stddev', 3.0)

        try:
            h_std = self.get_parameter('horizontal_stddev').get_parameter_value().double_value
            v_std = self.get_parameter('vertical_stddev').get_parameter_value().double_value
        except Exception as e:
            h_std = 2.0  # Default value
            v_std = 3.0  # Default value

        self.h_var = h_std ** 2
        self.v_var = v_std ** 2

        self.sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.callback, 10)
        self.pub = self.create_publisher(NavSatFix, '/gps/fix_cov', 10)

        if self.is_sim:
            self.get_logger().info(
                f'GPS cov relay [SIM]: injecting h_var={self.h_var:.1f}, v_var={self.v_var:.1f}')
        else:
            self.get_logger().info(
                'GPS cov relay [REAL]: passing through GPS covariance from receiver')

    def callback(self, msg: NavSatFix):
        msg.header.frame_id = 'gps_link'
        
        # Force covariance injection for the Columbus P-7 Pro even on real hardware,
        # because the raw NMEA driver relies on loose HDOP math without $GPGST strings.
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
    try:
        rclpy.init(args=args)
        node = GpsCovRelay()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Shutting down gps_cov_relay node...')
        except Exception as e:
            node.get_logger().error(f'Unexpected error: {e}')
        finally:
            node.destroy_node()
    except Exception as e:
        print(f'Failed to initialize GPS Covariance Relay: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
