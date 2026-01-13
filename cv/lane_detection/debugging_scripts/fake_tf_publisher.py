#!/usr/bin/env python3
"""
Fake TF Publisher for Lane Detection Standalone Testing

Purpose:
    Publishes the TF transforms required by lane_detection nodes
    when running without the full robot stack.

Published Transforms:
    - map -> odom
    - odom -> base_link
    - base_link -> left_camera_link_optical
    - base_link -> bottom_lidar_link

Usage:
    ros2 run lane_detection fake_tf_publisher
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import math


class FakeTFPublisher(Node):
    def __init__(self):
        super().__init__('fake_tf_publisher')
        
        # Static TF broadcaster for fixed transforms
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Dynamic TF broadcaster (for odom->base_link if needed)
        self.dynamic_broadcaster = TransformBroadcaster(self)
        
        # Publish static transforms immediately
        self.publish_static_transforms()
        
        # Timer for dynamic transforms (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_dynamic_transforms)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Fake TF Publisher Started!')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Publishing transforms:')
        self.get_logger().info('  - map -> odom (static)')
        self.get_logger().info('  - odom -> base_link (dynamic)')
        self.get_logger().info('  - base_link -> left_camera_link_optical (static)')
        self.get_logger().info('  - base_link -> bottom_lidar_link (static)')
        self.get_logger().info('=' * 60)

    def publish_static_transforms(self):
        """Publish static transforms that don't change."""
        now = self.get_clock().now().to_msg()
        transforms = []
        
        # map -> odom (identity)
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = now
        t_map_odom.header.frame_id = 'map'
        t_map_odom.child_frame_id = 'odom'
        t_map_odom.transform.rotation.w = 1.0
        transforms.append(t_map_odom)
        
        # base_link -> left_camera_link_optical
        # Camera is mounted facing forward, optical frame has Z forward, X right, Y down
        t_base_camera = TransformStamped()
        t_base_camera.header.stamp = now
        t_base_camera.header.frame_id = 'base_link'
        t_base_camera.child_frame_id = 'left_camera_link_optical'
        t_base_camera.transform.translation.x = 0.2  # 20cm in front of base
        t_base_camera.transform.translation.z = 0.3  # 30cm above base
        # Rotate to optical frame convention (Z forward, X right, Y down)
        # This is a -90° roll, then -90° yaw from base_link
        t_base_camera.transform.rotation.x = -0.5
        t_base_camera.transform.rotation.y = 0.5
        t_base_camera.transform.rotation.z = -0.5
        t_base_camera.transform.rotation.w = 0.5
        transforms.append(t_base_camera)
        
        # base_link -> bottom_lidar_link
        t_base_lidar = TransformStamped()
        t_base_lidar.header.stamp = now
        t_base_lidar.header.frame_id = 'base_link'
        t_base_lidar.child_frame_id = 'bottom_lidar_link'
        t_base_lidar.transform.translation.x = 0.3  # 30cm in front
        t_base_lidar.transform.translation.z = 0.15  # 15cm above ground
        t_base_lidar.transform.rotation.w = 1.0
        transforms.append(t_base_lidar)
        
        self.static_broadcaster.sendTransform(transforms)

    def publish_dynamic_transforms(self):
        """Publish dynamic transforms (odom -> base_link)."""
        now = self.get_clock().now().to_msg()
        
        # odom -> base_link (robot at origin, not moving)
        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = now
        t_odom_base.header.frame_id = 'odom'
        t_odom_base.child_frame_id = 'base_link'
        t_odom_base.transform.rotation.w = 1.0
        
        self.dynamic_broadcaster.sendTransform(t_odom_base)


def main(args=None):
    rclpy.init(args=args)
    node = FakeTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
