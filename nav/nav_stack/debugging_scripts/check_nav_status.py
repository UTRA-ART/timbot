#!/usr/bin/env python3
"""
Check nav_stack status - verify all required topics and TF are available.

Usage:
  ros2 run nav_stack check_nav_status.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import subprocess


class NavStatusChecker(Node):
    def __init__(self):
        super().__init__('nav_status_checker')
        
        self.odom_received = False
        self.scan_received = False
        
        # Subscribe to key topics
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/local', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_modified', self.scan_cb, 10)
        
        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer to check and report status
        self.timer = self.create_timer(2.0, self.check_status)
        self.check_count = 0
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Nav Stack Status Checker')
        self.get_logger().info('=' * 60)

    def odom_cb(self, msg):
        self.odom_received = True

    def scan_cb(self, msg):
        self.scan_received = True

    def check_status(self):
        self.check_count += 1
        self.get_logger().info(f'\n--- Status Check #{self.check_count} ---')
        
        # Check topics
        self.get_logger().info('Topics:')
        self.get_logger().info(f'  /odometry/local: {"✓ RECEIVING" if self.odom_received else "✗ NOT RECEIVING"}')
        self.get_logger().info(f'  /scan_modified:  {"✓ RECEIVING" if self.scan_received else "✗ NOT RECEIVING"}')
        
        # Check TF
        self.get_logger().info('\nTF Frames:')
        # try:
        #     result = subprocess.run(
        #         ['ros2', 'run', 'tf2_ros', 'tf2_echo', 'map', 'base_link'],
        #         capture_output=True, text=True, timeout=2)
        #     if 'Exception' in result.stderr or 'error' in result.stderr.lower():
        #         self.get_logger().info('  map -> base_link: ✗ NOT AVAILABLE')
        #     else:
        #         self.get_logger().info('  map -> base_link: ✓ AVAILABLE')
        # except subprocess.TimeoutExpired:
        #     self.get_logger().info('  map -> base_link: ✗ TIMEOUT')
        # except Exception as e:
        #     self.get_logger().info(f'  map -> base_link: ✗ ERROR ({e})')

        # Alternative TF check using tf2_ros Buffer to avoid mistaken timeout reports.
        try:
            ok = self.tf_buffer.can_transform(
                target_frame='map',
                source_frame='base_link',
                time=Time(),  
                timeout=Duration(seconds=0.5),
            )
            self.get_logger().info(f'  map -> base_link: {"✓ AVAILABLE" if ok else "✗ NOT AVAILABLE"}')
        except Exception as e:
            self.get_logger().info(f'  map -> base_link: ✗ ERROR ({e})')
        
        # Check Nav2 nodes
        self.get_logger().info('\nNav2 Nodes:')
        try:
            result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True, timeout=5)
            nodes = result.stdout.strip().split('\n')
            
            nav_nodes = ['planner_server', 'controller_server', 'bt_navigator', 'lifecycle_manager_navigation']
            for node in nav_nodes:
                found = any(node in n for n in nodes)
                self.get_logger().info(f'  {node}: {"✓ RUNNING" if found else "✗ NOT FOUND"}')
        except Exception as e:
            self.get_logger().info(f'  Error checking nodes: {e}')
        
        # Check action servers
        self.get_logger().info('\nAction Servers:')
        try:
            result = subprocess.run(['ros2', 'action', 'list'], capture_output=True, text=True, timeout=5)
            actions = result.stdout.strip().split('\n')
            
            if '/navigate_to_pose' in actions:
                self.get_logger().info('  /navigate_to_pose: ✓ AVAILABLE')
            else:
                self.get_logger().info('  /navigate_to_pose: ✗ NOT AVAILABLE')
        except Exception as e:
            self.get_logger().info(f'  Error checking actions: {e}')
        
        # Reset for next check
        self.odom_received = False
        self.scan_received = False
        
        # Summary
        self.get_logger().info('\n' + '=' * 60)
        if self.check_count >= 3:
            self.get_logger().info('Status check complete. Press Ctrl+C to exit.')


def main(args=None):
    rclpy.init(args=args)
    node = NavStatusChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
