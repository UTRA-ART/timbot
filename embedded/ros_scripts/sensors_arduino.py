#!/usr/bin/env python3
'''
2024-05-23

read from arduino and control brk and en of motors

Directory:electronics/src/sensors_arduino.py
Corresponding launch file: 

subscribes to:
- delay_restart
- enable_motors

publishes to:
- motors_stopped
- electronics_status
- electronics_status/details

params
- baud_rate
- ros_rate
- arduino_port
'''

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool


# messages
OK_MSG = "O"
TEMPERATURE_MSG = "T"
CURRENT_MSG = "C"
ALARM_MSG = "A"
USER_MSG = "U"

ERROR_MSGS = [TEMPERATURE_MSG, CURRENT_MSG, ALARM_MSG, USER_MSG]

 

class Sensors_Arduino(Node):
    def __init__(self):
        super().__init__('fault_monitor')

        self.motors_pub = self.create_publisher(Bool, "/motors_stopped", 10)
        self.status_pub = self.create_publisher(String, "/electronics_status/msg", 10)
        self.details_pub = self.create_publisher(String, "/electronics_status/details", 10)

        self.enable_sub = self.create_subscription(Bool, '/enable_motors', self.enable_cb, 10)

        self.declare_parameter('arduino_port')
        self.declare_parameter('baud_rate')
        self.declare_parameter('ros_rate')

        self.arduino_port = self.get_parameter('arduino_port').value
        self.BAUD_RATE = self.get_parameter('baud_rate').value
        self.ROS_RATE = self.get_parameter('ros_rate').value

        # settings
        self.restart_when_ok = True  # motors enabled
        self.last_restart_msg = False
    
    def enable_cb(self, enable_msg):
        if enable_msg.data:
            self.restart_when_ok = True 
        else:
            self.restart_when_ok = False 




if __name__ == '__main__':

    rclpy.init()
    node = Sensors_Arduino()

    # serial connection setup
    conn = serial.Serial(node.arduino_port, node.BAUD_RATE, timeout=1)
    conn.reset_input_buffer()

    # run
    while rclpy.ok():
        rclpy.spin_once(node)
        if conn.in_waiting > 0:
            try:
                line = conn.readline().decode('utf-8').rstrip()
        
                if line == OK_MSG:
                    if node.restart_when_ok:
                        motors_msg = Bool()
                        motors_msg.data = False
                        node.motors_pub.publish(motors_msg)

                        status_msg = String()
                        status_msg.data = f"{OK_MSG} ON"
                        node.status_pub.publish(status_msg)
                    else:
                        motors_msg = Bool()
                        motors_msg.data = True
                        node.motors_pub.publish(motors_msg)

                        status_msg = String()
                        status_msg.data = f"{OK_MSG} OFF"
                        node.status_pub.publish(status_msg)
                elif line in ERROR_MSGS:
                    motors_msg = Bool()
                    motors_msg.data = True
                    node.motors_pub.publish(motors_msg)

                    status_msg = String()
                    status_msg.data = line
                    node.status_pub.publish(status_msg)
                else:
                    details_msg = String()
                    details_msg.data = line
                    node.details_pub.publish(details_msg)
                
            except:
                pass
        
        if node.restart_when_ok != node.last_restart_msg:
            if node.restart_when_ok:
                conn.write(b'r')
            else:
                conn.write(b's')

            node.last_restart_msg = node.restart_when_ok
    
    node.destroy_node()
    rclpy.shutdown()

