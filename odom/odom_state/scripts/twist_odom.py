#!/usr/bin/env python3


'''
CURRENTLY UNUSED SCRIPT
Twist to TwistWithCovariance for EKF
Purpose:
    Converts velocity command messages (Twist) into a format suitable for
    feeding into an Extended Kalman Filter (TwistWithCovarianceStamped).
    This allows the EKF to utilize commanded velocities along with their
    associated uncertainties for improved state estimation.
Subscribes to:
    - twist_mux/cmd_vel (geometry_msgs/Twist): Velocity commands from the robot's controller
Publishes to:
    - motor_cmd_for_ekf (geometry_msgs/TwistWithCovarianceStamped): Velocity commands with covariance for EKF
Usage:
    ros2 run odom_state twist_odom
Note:   
    Not currently integrated into the main system. Provided as a utility for
    potential future use in enhancing EKF input data.
'''
import rclpy
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped

class Vel_Cmds_to_EKF(rclpy.Node):
    def __init__(self):
        # Initialize
        super().__init__('motor_cmd_to_ekf_pub')
        
        # Makes the subsciber
        self.subscription = self.create_subscription(Twist, "twist_mux/cmd_vel", self.callback)

        # Makes the publisher
        self.publisher = self.create_publisher(TwistWithCovarianceStamped, 'motor_cmd_for_ekf', 10)

    def callback(self, cmd_msg):
        # Creates a message
        self.msg_for_ekf = TwistWithCovarianceStamped()
        self.msg_for_ekf.twist.twist = cmd_msg
        self.msg_for_ekf.header.stamp = rclpy.get_clock.now()
        self.msg_for_ekf.header.frame_id = "base_link"

        self.msg_for_ekf.twist.covariance = [0]*36

        self.publisher.publish(self.msg_for_ekf)

if __name__ == '__main__':
    rclpy.init(args=None)

    node = Vel_Cmds_to_EKF()
    rclpy.spin(node)
    rclpy.shutdown()