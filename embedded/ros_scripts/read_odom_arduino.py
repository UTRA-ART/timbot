'''
2024-05-11

read from arduino

Directory: electronics/src/read_odom_arduino.py
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



class Read_Odom_Arduino(Node):
    def __init__(self):
        super().__init__("ticks_publisher")

        self.ticks_pub_r = self.create_publisher(Int32, "right_wheel/ticks", 10)
        self.ticks_pub_l = self.create_publisher(Int32, "left_wheel/ticks", 10)
        # Subscriptions were commented out in the old code
        #self.r_subscription = self.create_subscription(Float64, 'right_wheel/command', self.r_command_cb, 10)
        #self.l_subscription = self.create_subscription(Float64, 'left_wheel/command', self.l_command_cb, 10)
        self.declare_parameter('arduino_port')
        self.declare_parameter('baud_rate')
        self.declare_parameter('ros_rate')

        self.arduino_port = self.get_parameter('arduino_port').value
        self.BAUD_RATE = self.get_parameter('baud_rate').value
        self.ROS_RATE = self.get_parameter('ros_rate').value

        self.direction_r = 1
        self.direction_l = 1
    
    def r_command_cb(self, control_msg):
        if control_msg.data >= 0:
            self.direction_r = True
        else:
            self.direction_r = False

    def l_command_cb(self, control_msg):
        if control_msg.data >= 0:
            self.direction_l = False
        else:
            self.direction_l = True



if __name__ == '__main__':

    rclpy.init()

    node = Read_Odom_Arduino()

    # serial connection setup
    conn = serial.Serial(node.arduino_port, node.BAUD_RATE, timeout=1)
    conn.reset_input_buffer()

    l_val = 0
    r_val = 0

    while rclpy.ok():
        if conn.in_waiting > 0:
            try:
                line = conn.readline().decode('utf-8').rstrip()
                l_val, r_val = line[1:-1].split(',') # data in format <{left_count},{right_count}>

                left_msg = Int32()
                left_msg.data = node.direction_l * int(l_val)
                right_msg = Int32()
                right_msg.data = node.direction_r * int(r_val)

                node.ticks_pub_l.publish(left_msg)
                node.ticks_pub_r.publish(right_msg)
            except:
                pass
    
    node.destroy_node()
    rclpy.shutdown()
 

