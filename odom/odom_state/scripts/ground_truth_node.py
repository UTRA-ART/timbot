#!/usr/bin/env python3

import rclpy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry

class GroundTruth(rclpy.Node):
    def __init__(self):
        # Initialize
        super().__init__('ground_truth_pub')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.ground_truth_msg = Odometry()

        # Publisher
        self.publisher = self.create_publisher(Odometry, 'ground_truth_odom', 1)

        # Subscriber
        self.subscription = self.create_subscription(Odometry, "ground_truth/state", self.callback)

    def callback(self, gazebo_position):
        #transform coordinates with rotation of 90 degrees and translation offset

        self.gazebo_pose = tf2_geometry_msgs.PoseStamped()

        self.gazebo_pose.pose = gazebo_position.pose.pose

        self.gazebo_pose.header.frame_id = 'world'
        self.gazebo_pose.header.stamp = rclpy.get_clock.now()

        self.ground_truth_pose = self.tf_buffer.transform(self.gazebo_pose,'ground_truth', rclpy.duration.Duration(seconds=1.0))

        self.ground_truth_msg.pose.pose = self.ground_truth_pose.pose

        self.ground_truth_msg.header.stamp=rclpy.get_clock.now()
        self.ground_truth_msg.header.frame_id = 'ground_truth'

        self.publisher.publish(self.ground_truth_msg)

if __name__ == '__main__':
    rclpy.init(args=None)

    node = GroundTruth()
    rclpy.spin(node)
    rclpy.shutdown()