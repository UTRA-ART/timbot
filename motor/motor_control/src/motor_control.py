#!/usr/bin/env python3
'''
Subscribes to velocity command topics for each wheel and converts to appropriate RPi output
2026-02-28

Directory: /motor-control/src/motor_control.py

To do:
* adjust RATE
* adjust TIMEOUT

Subscribes to:
* /twist_mux/cmd_vel
* pause_navigation

Publishes to:
* /right_wheel/direction
* /left_wheel/direction
* /right_wheel/ticks (directionless count)
* /left_wheel/ticks (directionless count)
* debug
'''

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

import RPi.GPIO as gpio
import time
from math import pi


arduino_port = '/dev/ttyACM0'

BAUD_RATE = 115200

# controls
ROS_RATE = 30
PWM_FREQ = 512
TIMEOUT = 3000 # stop motors if TIMEOUT seconds have passed

# constants
WHEEL_RADIUS = 0.125    # update this with urdf
CIRCUMFERENCE = 2 * pi * WHEEL_RADIUS
WHEEL_BASE = 0.823976          # update this with urdf

VEL_MAX = 39    # 2.2352        # 5 mph = 2.2352 m/s

# conversion: duty cycle = A * rpm + B
A_left = 0.4385
B_left = -5.9086

A_left_small = 0.2016

def convert_speed_left(target):
    rpm = target * 60 / CIRCUMFERENCE
    dc = A_left * rpm + B_left
    return dc if dc > 5 else A_left_small * rpm

A_right = 0.4385
B_right = -5.9086

A_right_small = 0.2016

def convert_speed_right(target):
    rpm = target * 60 / CIRCUMFERENCE
    dc = A_right * rpm + B_right
    return dc if dc > 5 else A_right_small * rpm

# pins
LIGHT_PIN = 16      # (36)
R_IS_STOPPED = 24   # (18)
L_IS_STOPPED = 23   # (16)

R_SPEED_PIN = 26
R_DIR_PIN = 19
L_DIR_PIN = 13
L_SPEED_PIN = 6

# light variables
BLINK_INTERVAL = 0.5


class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # speed variables
        self.g_vx = 0.0
        self.g_wz = 0.0

        # motor variables
        self.right_speed = 0.0
        self.right_dir = True # True: CW, False: CCW
        self.left_speed = 0.0
        self.left_dir = True
        self.right_dir_last = True
        self.left_dir_last = False
        
        self.vl=0
        self.vr=0

        # timing
        self.rostime_last = 0.0
        self.current_time = 0.0
        self.lighttime_last = 0.0

        # direction multipliers for ticks
        self.direction_r = 1
        self.direction_l = 1

        # mode: False = manual (light solid), True = autonomous (light flashes)
        self.mode = False
        self.light_state = True

        # Whether to run motor control code (alternates to run at 15 Hz)
        self.run_motor_control = True

        # serial connection setup
        self.conn = serial.Serial(arduino_port, BAUD_RATE, timeout=1)
        self.conn.reset_input_buffer()

        # GPIO setup
        self.get_logger().info("Setting up pins...")
        gpio.setmode(gpio.BCM)
        gpio.setup(R_DIR_PIN, gpio.OUT, initial=gpio.LOW)
        gpio.setup(L_DIR_PIN, gpio.OUT, initial=gpio.LOW)
        gpio.setup(LIGHT_PIN, gpio.OUT, initial=gpio.LOW)
        gpio.setup(R_IS_STOPPED, gpio.OUT, initial=gpio.LOW)
        gpio.setup(L_IS_STOPPED, gpio.OUT, initial=gpio.LOW)

        gpio.setup(R_SPEED_PIN, gpio.OUT)
        self.r_speed_pin = gpio.PWM(R_SPEED_PIN, PWM_FREQ)
        self.r_speed_pin.start(0)
        gpio.setup(L_SPEED_PIN, gpio.OUT)
        self.l_speed_pin = gpio.PWM(L_SPEED_PIN, PWM_FREQ)
        self.l_speed_pin.start(0)

        # subscribers
        self.get_logger().info("Subscribing to topics...")
        self.create_subscription(Twist, '/cmd_vel', self.target_cb, 10)
        self.create_subscription(Bool, 'pause_navigation', self.mode_cb, 10)

        # publishers
        self.ticks_pub_r = self.create_publisher(Int32, '/right_wheel/ticks', 10)
        self.ticks_pub_l = self.create_publisher(Int32, '/left_wheel/ticks', 10)
        self.right_dir_pub = self.create_publisher(Bool, '/right_wheel/direction', 1)
        self.left_dir_pub = self.create_publisher(Bool, '/left_wheel/direction', 1)
        self.debug_pub = self.create_publisher(String, 'debug', 10)

        # timer (runs at ROS_RATE Hz)
        timer_period = 1.0 / ROS_RATE
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Running")

    def target_cb(self, target_msg):
        self.rostime_last = time.time()
        self.g_vx = min(target_msg.linear.x, VEL_MAX)
        self.g_wz = target_msg.angular.z

    def mode_cb(self, mode_msg):
        self.mode = mode_msg.data

    def timer_callback(self):
        
        
        self.current_time = time.time()
        # Arduino read code, runs at 30 Hz
        if self.conn.in_waiting > 0:
            try:
                raw_bytes = self.conn.read(self.conn.in_waiting)
                raw_data = raw_bytes.decode('utf-8', errors='ignore')
                # self.get_logger().info(f"Raw bytes: {raw_bytes}")
                
                packets = []
                for p in raw_data.split('<'):
                    if '>' in p:
                        packets.append(p.split('>')[0])
                        
                if packets:
                    latest_packet = packets[-1]
                    l_val, r_val = latest_packet.split(',')  # data in format <{left_count},{right_count}>
                    # Update direction multipliers
                    if self.right_dir:
                        self.direction_r = 1
                    else:
                        self.direction_r = -1

                    if not self.left_dir:
                        self.direction_l = 1
                    else:
                        self.direction_l = -1

                    # Publish directionless tick counts; odom node applies direction using
                    # /left_wheel/direction and /right_wheel/direction topics.
                    l_msg = Int32()
                    l_msg.data = int(l_val)
                    self.ticks_pub_l.publish(l_msg)
                    # self.get_logger().info(f"Left Ticks: {l_msg.data}")

                    r_msg = Int32()
                    r_msg.data = int(r_val)
                    self.ticks_pub_r.publish(r_msg)
            except Exception:
                pass

        # Motor control code, runs at 15 Hz (every other cycle)
        if self.run_motor_control:
            if self.current_time - self.rostime_last >= TIMEOUT and not self.mode:
                # command has not been received in some time
                # something may be wrong -> stop the motors
                gpio.output(R_DIR_PIN, gpio.LOW)
                gpio.output(L_DIR_PIN, gpio.LOW)
                self.r_speed_pin.ChangeDutyCycle(0)
                self.l_speed_pin.ChangeDutyCycle(0)
            else:
                # calculate speeds for each wheel
                # right speed
                vr_target = self.g_vx + (WHEEL_BASE * self.g_wz) / 2
                self.vr=self.vr+(vr_target-self.vr)*0.2
                if self.vr < 0.05 and self.vr > -0.05:
                    right_duty_cycle = 0.0
                elif self.vr > 0:
                    right_duty_cycle = convert_speed_right(self.vr)
                    self.right_dir = False
                else:
                    right_duty_cycle = convert_speed_right(-self.vr)
                    self.right_dir = True
                self.right_speed = min(max(right_duty_cycle, 0), VEL_MAX)

                # left speed
                vl_target = self.g_vx - (WHEEL_BASE * self.g_wz) / 2
                self.vl = self.vl+(vl_target-self.vl)*0.2
                if self.vl < 0.05 and self.vl > -0.05:
                    left_duty_cycle = 0.0
                elif self.vl > 0:
                    left_duty_cycle = convert_speed_left(self.vl)
                    self.left_dir = True
                else:
                    left_duty_cycle = convert_speed_left(-self.vl)
                    self.left_dir = False
                self.left_speed = min(max(left_duty_cycle, 0), VEL_MAX)

                # write speed and direction pins
                gpio.output(R_DIR_PIN, self.right_dir)
                gpio.output(L_DIR_PIN, self.left_dir)

                # pin output if motor is stopped
                gpio.output(R_IS_STOPPED, self.right_speed != 0)
                gpio.output(L_IS_STOPPED, self.left_speed != 0)

                test_msg = f'Right: {self.right_speed}, {self.right_dir}, Left: {self.left_speed}, {self.left_dir}'
                self.get_logger().info(test_msg)

                debug_msg1 = String()
                debug_msg1.data = f"rpm: {self.vr * 60 / CIRCUMFERENCE}, {self.vl * 60 / CIRCUMFERENCE}"
                self.debug_pub.publish(debug_msg1)

                debug_msg2 = String()
                debug_msg2.data = f"{right_duty_cycle}\t{left_duty_cycle}"
                self.debug_pub.publish(debug_msg2)

                debug_msg3 = String()
                debug_msg3.data = test_msg
                self.debug_pub.publish(debug_msg3)

                # set speed
                self.r_speed_pin.ChangeDutyCycle(self.right_speed)
                self.l_speed_pin.ChangeDutyCycle(self.left_speed)

                # publish direction
                r_dir_msg = Bool()
                r_dir_msg.data = self.right_dir
                self.right_dir_pub.publish(r_dir_msg)

                l_dir_msg = Bool()
                l_dir_msg.data = not self.left_dir
                self.left_dir_pub.publish(l_dir_msg)

            # control light
            if not self.mode:
                # autonomous mode; flashing
                if self.current_time - self.lighttime_last >= BLINK_INTERVAL:
                    self.lighttime_last = self.current_time
                    self.light_state = not self.light_state
                    gpio.output(LIGHT_PIN, self.light_state)
            else:
                # manual mode; solid
                gpio.output(LIGHT_PIN, True)

        # alternate to achieve 15 Hz motor control
        self.run_motor_control = not self.run_motor_control

    def node_cleanup(self):
        '''set all pins to 0, then call cleanup function'''
        gpio.output(R_DIR_PIN, gpio.LOW)
        gpio.output(L_DIR_PIN, gpio.LOW)
        self.r_speed_pin.ChangeDutyCycle(0)
        self.l_speed_pin.ChangeDutyCycle(0)
        gpio.output(LIGHT_PIN, gpio.LOW)
        gpio.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControl()
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