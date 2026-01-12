#!/usr/bin/env python3
"""
Send navigation goals to nav_stack for testing.

Usage:
  ros2 run nav_stack send_nav_goal.py              # Default: go to (2, 0)
  ros2 run nav_stack send_nav_goal.py --x 5 --y 3  # Custom goal
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import argparse
import sys
import math


class NavGoalSender(Node):
    def __init__(self, x, y, yaw):
        super().__init__('nav_goal_sender')
        
        self.goal_x = x
        self.goal_y = y
        self.goal_yaw = yaw
        
        # Action client for NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Nav Goal Sender')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Target: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad')
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        
        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            self.get_logger().error('Make sure nav_stack is running:')
            self.get_logger().error('  ros2 launch nav_stack move_base.launch.py use_sim_time:=false')
            return
        
        self.get_logger().info('Action server available! Sending goal...')
        self.send_goal()

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(self.goal_yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(self.goal_yaw / 2.0)
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted! Navigating...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        distance = feedback.distance_remaining
        self.get_logger().info(
            f'Progress: pos=({current_pose.position.x:.2f}, {current_pose.position.y:.2f}), '
            f'distance_remaining={distance:.2f}m')

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('=' * 60)
            self.get_logger().info('Navigation SUCCEEDED!')
            self.get_logger().info('=' * 60)
        else:
            self.get_logger().warn(f'Navigation finished with status: {status}')
        
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Send navigation goal')
    parser.add_argument('--x', type=float, default=2.0, help='Goal X position')
    parser.add_argument('--y', type=float, default=0.0, help='Goal Y position')
    parser.add_argument('--yaw', type=float, default=0.0, help='Goal yaw (radians)')
    
    # Parse known args to handle ROS args
    args, _ = parser.parse_known_args()
    
    rclpy.init()
    node = NavGoalSender(args.x, args.y, args.yaw)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
