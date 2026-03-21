#!/usr/bin/env python3
'''
2024-05-11

Script to read odometry ticks from Arduino

Directory: embedded/ros_scripts/read_odom_arduino.py
Launch file: motor/motor_control/launch/motor_control_odom.launch

subscribes to:
- left_wheel/command    (Float64)
- right_wheel/command

publishes to:
- left_wheel/ticks      (Int32)
- right_wheel/ticks

params
- baud_rate
- ros_rate
- arduino_port
'''

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float64


class ReadOdomArduino(Node):
    def __init__(self):
        super().__init__("ticks_publisher")

        # publishers
        self.ticks_pub_r = self.create_publisher(Int32, "/right_wheel/ticks", 10)
        self.ticks_pub_l = self.create_publisher(Int32, "/left_wheel/ticks", 10)

        # subscribers
        self.create_subscription(Float64, '/right_wheel/command', self.r_command_cb, 10)
        self.create_subscription(Float64, '/left_wheel/command', self.l_command_cb, 10)

        # parameters
        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('ros_rate', 30)

        arduino_port = self.get_parameter('arduino_port').value
        baud_rate = self.get_parameter('baud_rate').value
        ros_rate = self.get_parameter('ros_rate').value

        # direction multipliers for ticks
        self.direction_r = 1
        self.direction_l = 1

        # serial connection setup
        self.conn = serial.Serial(arduino_port, baud_rate, timeout=1)
        self.conn.reset_input_buffer()

        # timer
        timer_period = 1.0 / ros_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Running")

    def r_command_cb(self, control_msg):
        self.direction_r = 1 if control_msg.data >= 0 else -1

    def l_command_cb(self, control_msg):
        self.direction_l = -1 if control_msg.data >= 0 else 1

    def timer_callback(self):
        if self.conn.in_waiting > 0:
            try:
                line = self.conn.readline().decode('utf-8').rstrip()
                l_val, r_val = line[1:-1].split(',')  # data in format <{left_count},{right_count}>

                left_msg = Int32()
                left_msg.data = self.direction_l * int(l_val)
                self.ticks_pub_l.publish(left_msg)

                right_msg = Int32()
                right_msg.data = self.direction_r * int(r_val)
                self.ticks_pub_r.publish(right_msg)
            except Exception:
                pass

    def node_cleanup(self):
        self.conn.close()


def main(args=None):
    rclpy.init(args=args)
    node = ReadOdomArduino()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.node_cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()