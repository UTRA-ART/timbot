#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleMotorEcho(Node):
    def __init__(self):
        super().__init__('simple_motor_echo')
        
        # Just subscribe to cmd_vel and print it
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('Simple Motor Echo started - listening to /cmd_vel')

    def cmd_vel_callback(self, msg):
        """Just print the received command"""
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.get_logger().info(
                f'Received: Forward={msg.linear.x:.2f} m/s, Turn={msg.angular.z:.2f} rad/s'
            )

def main(args=None):
    rclpy.init(args=args)
    motor_echo = SimpleMotorEcho()
    
    try:
        rclpy.spin(motor_echo)
    except KeyboardInterrupt:
        motor_echo.get_logger().info('Motor Echo shutting down')
    finally:
        motor_echo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()