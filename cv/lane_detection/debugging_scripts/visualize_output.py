#!/usr/bin/env python3
"""
Lane Detection Output Visualizer

Purpose:
    Subscribes to lane detection output topics and visualizes results.
    Useful for debugging the full ROS pipeline.

Subscribed Topics:
    - /cv/model_output (sensor_msgs/Image): Raw model output mask
    - /cv/lane_detections (FloatArray): 3D lane points
    - /cv/lane_detections_cloud (PointCloud2): Lane point cloud

Usage:
    # Visualize model output mask
    ros2 run lane_detection visualize_output

    # With additional options
    ros2 run lane_detection visualize_output --ros-args \
        -p save_images:=true \
        -p output_dir:=/path/to/save
"""

import os
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

# Try to import custom message (may not be available outside ROS build)
try:
    from lane_detection.msg import FloatArray
    HAS_CUSTOM_MSG = True
except ImportError:
    HAS_CUSTOM_MSG = False
    print("Warning: FloatArray message not available, some features disabled")


class LaneVisualizer(Node):
    """Visualizes lane detection outputs."""

    def __init__(self):
        super().__init__('lane_visualizer')
        
        # Parameters
        self.declare_parameter('save_images', False)
        self.declare_parameter('output_dir', './lane_vis_output')
        self.declare_parameter('show_fps', True)
        
        self.save_images = self.get_parameter('save_images').value
        self.output_dir = self.get_parameter('output_dir').value
        self.show_fps = self.get_parameter('show_fps').value
        
        if self.save_images:
            os.makedirs(self.output_dir, exist_ok=True)
        
        self.bridge = CvBridge()
        
        # Timing
        self.last_time = time.time()
        self.fps = 0.0
        self.frame_count = 0
        
        # Latest data
        self.latest_mask = None
        self.latest_points = []
        
        # Subscribers
        self.mask_sub = self.create_subscription(
            Image, '/cv/model_output', self.mask_callback, qos_profile_sensor_data)
        
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/cv/lane_detections_cloud', self.cloud_callback, 10)
        
        if HAS_CUSTOM_MSG:
            self.detections_sub = self.create_subscription(
                FloatArray, '/cv/lane_detections', self.detections_callback, 10)
        
        # Timer for display update
        self.timer = self.create_timer(0.05, self.display_callback)  # 20 Hz
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Lane Detection Visualizer Started!')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  - /cv/model_output')
        self.get_logger().info('  - /cv/lane_detections_cloud')
        if HAS_CUSTOM_MSG:
            self.get_logger().info('  - /cv/lane_detections')
        self.get_logger().info('Press "q" in the OpenCV window to quit')
        self.get_logger().info('=' * 60)

    def mask_callback(self, msg):
        """Handle incoming mask images."""
        try:
            self.latest_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Update FPS
            current_time = time.time()
            dt = current_time - self.last_time
            if dt > 0:
                self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)
            self.last_time = current_time
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error processing mask: {e}')

    def cloud_callback(self, msg):
        """Handle incoming point cloud."""
        try:
            # Extract points from PointCloud2
            # This is a simplified extraction - full implementation would use point_cloud2 lib
            self.get_logger().debug(f'Received point cloud with {msg.width * msg.height} points')
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    def detections_callback(self, msg):
        """Handle incoming FloatArray detections."""
        try:
            points = []
            if msg.lists:
                for point in msg.lists[0].elements:
                    points.append((point.x, point.y, point.z))
            self.latest_points = points
            self.get_logger().debug(f'Received {len(points)} detection points')
        except Exception as e:
            self.get_logger().error(f'Error processing detections: {e}')

    def display_callback(self):
        """Update visualization display."""
        if self.latest_mask is None:
            return
        
        # Create visualization
        mask = self.latest_mask.copy()
        
        # Ensure 3 channels for display
        if len(mask.shape) == 2:
            vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        else:
            vis = mask
        
        # Colorize the mask (make detected lanes green)
        if len(mask.shape) == 2:
            lane_mask = mask > 0
            colored = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
            colored[lane_mask, 1] = 255  # Green for lanes
            vis = colored
        
        # Add info overlay
        if self.show_fps:
            cv2.putText(vis, f'FPS: {self.fps:.1f}', (10, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(vis, f'Frame: {self.frame_count}', (10, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Show number of detection points
        if self.latest_points:
            cv2.putText(vis, f'Points: {len(self.latest_points)}', (10, 75),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Display
        cv2.imshow('Lane Detection Output', vis)
        
        # Save if enabled
        if self.save_images:
            filepath = os.path.join(self.output_dir, f'frame_{self.frame_count:06d}.png')
            cv2.imwrite(filepath, vis)
        
        # Check for quit
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.get_logger().info('Quit requested')
            raise SystemExit()


def main(args=None):
    rclpy.init(args=args)
    node = LaneVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
