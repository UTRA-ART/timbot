#!/usr/bin/env python3
"""
Ground Truth Odometry Publisher for Gazebo Simulation

Purpose:
    Transforms Gazebo's perfect position data from simulator coordinate frame
    to the robot's coordinate frame. Provides "perfect" odometry for comparison
    against estimated odometry during testing.

Subscribes to:
    - ground_truth/state (nav_msgs/Odometry): Raw position from Gazebo

Publishes to:
    - ground_truth_odom (nav_msgs/Odometry): Transformed ground truth position

Usage:
    ros2 run odom_state ground_truth_node
    (Only useful in Gazebo simulation, not on real robot)
"""

import rclpy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry

class GroundTruth(rclpy.Node):
    def __init__(self):
        super().__init__('ground_truth_pub')
        
        # Set up TF2 listener to handle coordinate frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.ground_truth_msg = Odometry()

        # Publish transformed ground truth to this topic
        self.publisher = self.create_publisher(Odometry, 'ground_truth_odom', 1)

        # Subscribe to Gazebo's raw ground truth topic
        self.subscription = self.create_subscription(Odometry, "ground_truth/state", self.callback, 1)

    def callback(self, gazebo_position):
        """Transform Gazebo's world frame coordinates to robot's ground_truth frame"""
        
        # Convert Gazebo odometry to PoseStamped for TF transformation
        self.gazebo_pose = tf2_geometry_msgs.PoseStamped()

        self.gazebo_pose.pose = gazebo_position.pose.pose

        self.gazebo_pose.header.frame_id = 'world'
        self.gazebo_pose.header.stamp = rclpy.get_clock.now()

        # Apply TF transform from 'world' to 'ground_truth' frame
        self.ground_truth_pose = self.tf_buffer.transform(
            self.gazebo_pose, 'ground_truth', rclpy.duration.Duration(seconds=1.0)
        )

        # Build output odometry message with transformed pose
        self.ground_truth_msg.pose.pose = self.ground_truth_pose.pose
        self.ground_truth_msg.header.stamp = rclpy.get_clock.now()
        self.ground_truth_msg.header.frame_id = 'ground_truth'

        # Publish transformed ground truth
        self.publisher.publish(self.ground_truth_msg)

if __name__ == '__main__':
    rclpy.init(args=None)

    node = GroundTruth()
    rclpy.spin(node)
    rclpy.shutdown()