#!/usr/bin/env python3
import os
import math

import cv2
import numpy as np
import torch

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import Header

from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from sensor_msgs_py import point_cloud2

import message_filters


class CVModelInferencer(Node):
    def __init__(self):
        super().__init__('lane_detection_model_inference')

        # Publishers — only PointCloud2 for nav and debug image
        self.pub_pt = self.create_publisher(PointCloud2, 'cv/lane_detections_cloud', 10)
        self.pub_raw = self.create_publisher(Image, 'cv/model_output', 10)

        self.bridge = CvBridge()

        share_dir = get_package_share_directory('lane_detection')
        self.model_path = os.path.join(share_dir, 'models', 'best_model_int8.pt')

        # Get the parameter to decide between deep learning and classical
        self.declare_parameter('lane_detection_mode', 0)
        self.classical_mode = int(self.get_parameter('lane_detection_mode').value)

        self.Inference = None
        self.lane_detection = None

        if self.classical_mode == 1:
            from classical_lane_detection import lane_detection
            self.lane_detection = lane_detection
            self.get_logger().info("Lane Detection node initialized with CLASSICAL...")
        else:
            self.Inference = YOLO(self.model_path)
            self.get_logger().info(
                f"Lane Detection node initialized with DEEP LEARNING...\n"
                f"CUDA status: {torch.cuda.is_available()}"
            )

        # Camera intrinsics — populated from CameraInfo
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

    def run(self):
        # Subscribe to camera info once to get intrinsics
        self.create_subscription(
            CameraInfo,
            '/zed_node/left/camera_info',
            self.camera_info_callback,
            10
        )

        # Synchronized subscriptions to RGB + depth
        rgb_sub = message_filters.Subscriber(self, Image, '/image', qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(self, Image, '/zed_node/left/depth_image', qos_profile=qos_profile_sensor_data)

        sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=5, slop=0.1
        )
        sync.registerCallback(self.process_image)

        self.get_logger().info("Waiting for camera info and synchronized RGB + depth images...")
        rclpy.spin(self)

    def camera_info_callback(self, msg):
        """Extract camera intrinsics from CameraInfo message."""
        if self.fx is not None:
            return  # Already have intrinsics
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info(
            f"Camera intrinsics received: fx={self.fx:.1f}, fy={self.fy:.1f}, "
            f"cx={self.cx:.1f}, cy={self.cy:.1f}"
        )

    def process_image(self, rgb_data, depth_data):
        if self.fx is None:
            self.get_logger().warn("No camera intrinsics yet, skipping frame", throttle_duration_sec=5.0)
            return

        raw = self.bridge.imgmsg_to_cv2(rgb_data, desired_encoding='passthrough')
        if raw is None:
            return

        # Convert depth to numpy array (float32, meters)
        depth_img = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')

        # Resize input image to model input size
        input_img = raw.copy()
        input_img = cv2.resize(input_img, (330, 180))
        input_img = input_img[:, :, :3]

        # Run inference
        output = None
        if self.classical_mode:
            output = self.lane_detection(input_img)
            mask = np.where(output > 0.5, 1., 0.)
            output = (mask * 255).astype(np.uint8)
        else:
            result = self.Inference(input_img, verbose=False)
            confidence_threshold = 0.5
            output_image = np.zeros_like(input_img[:, :, 0], dtype=np.uint8)

            if result and result[0].masks:
                for k in range(len(result[0].masks)):
                    mask_data = result[0].masks[k].data
                    mask = np.array(mask_data.cpu() if torch.cuda.is_available() else mask_data)
                    label = result[0].names[int(result[0].boxes[k].cls)]

                    if float(result[0].boxes[k].conf) > confidence_threshold:
                        if label == 'lane':
                            img = np.where(mask > 0.5, 255, 0).astype(np.uint8)
                            img = cv2.resize(img.squeeze(), (output_image.shape[1], output_image.shape[0]))
                            output_image = np.maximum(output_image, img)

            output = output_image

        # Publish debug mask image
        img_msg = self.bridge.cv2_to_imgmsg(output, encoding='passthrough')
        img_msg.header.stamp = rgb_data.header.stamp
        self.pub_raw.publish(img_msg)

        # Resize depth to match the model output size (330x180)
        depth_resized = cv2.resize(depth_img, (330, 180), interpolation=cv2.INTER_NEAREST)

        # Scale intrinsics to the resized resolution
        h_orig, w_orig = depth_img.shape[:2]
        sx = 330.0 / w_orig
        sy = 180.0 / h_orig
        fx = self.fx * sx
        fy = self.fy * sy
        cx = self.cx * sx
        cy = self.cy * sy

        # Build 3D points from lane pixels using depth + intrinsics
        lane_pixels = np.where(output == 255)
        cloud = []
        for v, u in zip(lane_pixels[0], lane_pixels[1]):
            d = float(depth_resized[v, u])
            if not math.isfinite(d) or d <= 0.0 or d > 10.0:
                continue
            # Optical frame: Z forward, X right, Y down
            x = (u - cx) * d / fx
            y = (v - cy) * d / fy
            z = d
            cloud.append((x, y, z))

        # Publish PointCloud2
        pt_header = Header(frame_id='left_camera_link_optical')
        pt_header.stamp = rgb_data.header.stamp
        pt_cloud = point_cloud2.create_cloud_xyz32(pt_header, cloud)
        self.pub_pt.publish(pt_cloud)


def main(args=None):
    rclpy.init(args=args)
    wrapper = CVModelInferencer()
    try:
        wrapper.run()
    except KeyboardInterrupt:
        pass
    finally:
        wrapper.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
