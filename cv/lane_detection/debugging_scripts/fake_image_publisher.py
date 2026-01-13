#!/usr/bin/env python3
"""
Fake Image Publisher for lane_detection Testing

Purpose:
    Publishes synthetic or test images to test the lane_detection package
    without requiring actual camera hardware or ZED camera.

Published Topics:
    - /image (sensor_msgs/Image): Fake camera images at configurable rate

Modes:
    1. synthetic: Generates synthetic lane images with white lines
    2. file: Loads images from a directory
    3. video: Loads frames from a video file
    4. solid: Publishes a solid colored image (for baseline testing)

Usage:
    # Synthetic lane images (default)
    ros2 run lane_detection fake_image_publisher --ros-args -p mode:=synthetic

    # Load from directory
    ros2 run lane_detection fake_image_publisher --ros-args \
        -p mode:=file -p image_dir:=/path/to/images

    # Load from video
    ros2 run lane_detection fake_image_publisher --ros-args \
        -p mode:=video -p video_path:=/path/to/video.mp4

Data Flow:
    ┌─────────────────────────────────────────────────────────────────────┐
    │                     THIS SCRIPT PUBLISHES                           │
    ├─────────────────────────────────────────────────────────────────────┤
    │                                                                     │
    │  /image ──► lane_detection_inference ──► /cv/lane_detections       │
    │                        │                                            │
    │                        ├──► /cv/model_output                        │
    │                        └──► /cv/lane_detections_cloud               │
    │                                                                     │
    └─────────────────────────────────────────────────────────────────────┘
"""

import os
import glob
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class FakeImagePublisher(Node):
    """Publishes fake camera images for testing lane_detection package."""

    def __init__(self):
        super().__init__('fake_image_publisher')

        # Declare parameters
        self.declare_parameter('mode', 'synthetic')  # synthetic, file, video, solid
        self.declare_parameter('image_dir', '')  # directory with images
        self.declare_parameter('video_path', '')  # path to video file
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 360)
        self.declare_parameter('loop', True)  # loop through images/video

        # Lane generation parameters (for synthetic mode)
        self.declare_parameter('lane_color', [255, 255, 255])  # BGR white
        self.declare_parameter('lane_width', 20)
        self.declare_parameter('num_lanes', 2)
        self.declare_parameter('add_noise', True)
        self.declare_parameter('add_barrel', False)  # Add orange barrel to test filtering

        # Get parameters
        self.mode = self.get_parameter('mode').value
        self.image_dir = self.get_parameter('image_dir').value
        self.video_path = self.get_parameter('video_path').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.width = self.get_parameter('image_width').value
        self.height = self.get_parameter('image_height').value
        self.loop = self.get_parameter('loop').value
        self.lane_color = self.get_parameter('lane_color').value
        self.lane_width = self.get_parameter('lane_width').value
        self.num_lanes = self.get_parameter('num_lanes').value
        self.add_noise = self.get_parameter('add_noise').value
        self.add_barrel = self.get_parameter('add_barrel').value

        # Publisher
        self.image_pub = self.create_publisher(Image, '/image', 10)
        self.bridge = CvBridge()

        # Mode-specific setup
        self.images = []
        self.image_idx = 0
        self.video_cap = None
        self.frame_count = 0

        self._setup_mode()

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Fake Image Publisher Started!')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Mode: {self.mode}')
        self.get_logger().info(f'Image size: {self.width}x{self.height}')
        self.get_logger().info(f'Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Publishing to: /image')
        self.get_logger().info('=' * 60)

    def _setup_mode(self):
        """Setup based on selected mode."""
        if self.mode == 'file':
            if not self.image_dir or not os.path.isdir(self.image_dir):
                self.get_logger().error(f'Invalid image directory: {self.image_dir}')
                self.get_logger().info('Falling back to synthetic mode')
                self.mode = 'synthetic'
            else:
                patterns = ['*.png', '*.jpg', '*.jpeg', '*.bmp']
                for pattern in patterns:
                    self.images.extend(glob.glob(os.path.join(self.image_dir, pattern)))
                self.images.sort()
                self.get_logger().info(f'Loaded {len(self.images)} images from {self.image_dir}')

        elif self.mode == 'video':
            if not self.video_path or not os.path.isfile(self.video_path):
                self.get_logger().error(f'Invalid video path: {self.video_path}')
                self.get_logger().info('Falling back to synthetic mode')
                self.mode = 'synthetic'
            else:
                self.video_cap = cv2.VideoCapture(self.video_path)
                if not self.video_cap.isOpened():
                    self.get_logger().error('Failed to open video file')
                    self.mode = 'synthetic'
                else:
                    self.get_logger().info(f'Opened video: {self.video_path}')

    def generate_synthetic_image(self) -> np.ndarray:
        """Generate a synthetic image with lane markings."""
        # Create dark gray road background
        img = np.full((self.height, self.width, 3), 80, dtype=np.uint8)

        # Add some variation to simulate road texture
        if self.add_noise:
            noise = np.random.randint(-10, 10, (self.height, self.width, 3), dtype=np.int16)
            img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        # Draw lanes with perspective (converging toward horizon)
        horizon_y = int(self.height * 0.4)  # Horizon at 40% from top
        
        # Lane positions at bottom of image
        lane_spacing = self.width // (self.num_lanes + 1)
        
        for i in range(self.num_lanes):
            # Bottom position of lane
            bottom_x = lane_spacing * (i + 1)
            
            # Top position (converge toward center at horizon)
            center_x = self.width // 2
            top_x = center_x + (bottom_x - center_x) * 0.3
            
            # Draw lane line as polygon
            lane_color = tuple(self.lane_color)
            pts = np.array([
                [bottom_x - self.lane_width // 2, self.height],
                [bottom_x + self.lane_width // 2, self.height],
                [int(top_x + self.lane_width // 4), horizon_y],
                [int(top_x - self.lane_width // 4), horizon_y]
            ], np.int32)
            cv2.fillPoly(img, [pts], lane_color)

        # Add dashed center line
        center_x = self.width // 2
        for y in range(horizon_y, self.height, 40):
            y_ratio = (y - horizon_y) / (self.height - horizon_y)
            x = int(center_x)
            dash_width = int(5 + 10 * y_ratio)
            dash_height = 20
            if y + dash_height < self.height:
                cv2.rectangle(img, 
                             (x - dash_width // 2, y),
                             (x + dash_width // 2, y + dash_height),
                             (255, 255, 0), -1)  # Yellow dashed line

        # Optionally add orange barrel
        if self.add_barrel:
            barrel_x = self.width // 3
            barrel_y = int(self.height * 0.6)
            barrel_w = 30
            barrel_h = 50
            # Orange barrel with stripes
            cv2.rectangle(img, 
                         (barrel_x, barrel_y),
                         (barrel_x + barrel_w, barrel_y + barrel_h),
                         (0, 140, 255), -1)  # Orange
            # Add white stripes
            for stripe_y in range(barrel_y, barrel_y + barrel_h, 15):
                cv2.rectangle(img,
                             (barrel_x, stripe_y),
                             (barrel_x + barrel_w, stripe_y + 5),
                             (255, 255, 255), -1)

        # Add sky (blue gradient at top)
        for y in range(horizon_y):
            ratio = y / horizon_y
            sky_color = (int(180 + 40 * ratio), int(130 + 50 * ratio), int(80 + 40 * ratio))
            img[y, :] = sky_color

        # Animate the lanes slightly (simulate forward motion)
        self.frame_count += 1

        return img

    def timer_callback(self):
        """Publish an image based on the current mode."""
        img = None

        if self.mode == 'synthetic':
            img = self.generate_synthetic_image()

        elif self.mode == 'solid':
            # Simple solid gray image
            img = np.full((self.height, self.width, 3), 128, dtype=np.uint8)

        elif self.mode == 'file':
            if self.images:
                img = cv2.imread(self.images[self.image_idx])
                if img is not None:
                    img = cv2.resize(img, (self.width, self.height))
                self.image_idx += 1
                if self.image_idx >= len(self.images):
                    if self.loop:
                        self.image_idx = 0
                    else:
                        self.get_logger().info('Finished all images')
                        return

        elif self.mode == 'video':
            if self.video_cap is not None:
                ret, img = self.video_cap.read()
                if not ret:
                    if self.loop:
                        self.video_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        ret, img = self.video_cap.read()
                    if not ret:
                        self.get_logger().info('Finished video')
                        return
                if img is not None:
                    img = cv2.resize(img, (self.width, self.height))

        if img is not None:
            # Convert BGR to RGB for ROS
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            msg = self.bridge.cv2_to_imgmsg(img_rgb, encoding='rgb8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'left_camera_link_optical'
            self.image_pub.publish(msg)

    def destroy_node(self):
        if self.video_cap is not None:
            self.video_cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FakeImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
