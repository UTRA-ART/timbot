#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class DepthToPointCloud(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud')

        self.declare_parameter('depth_topic', '/zed_node/left/depth_image')
        self.declare_parameter('camera_info_topic', '/zed_node/left/camera_info')
        self.declare_parameter('pointcloud_topic', '/zed_node/left/depth_points')
        self.declare_parameter('frame_id', 'left_camera_link_optical')
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 20.0)
        self.declare_parameter('pixel_stride', 2)
        self.declare_parameter('filter_by_height', True)
        self.declare_parameter('min_height', 0.05)
        self.declare_parameter('max_height', 2.0)
        self.declare_parameter('camera_pitch_deg', 25.0)
        self.declare_parameter('camera_offset_x', 0.222173)
        self.declare_parameter('camera_offset_y', 0.061524)
        self.declare_parameter('camera_offset_z', 0.71)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.frame_id_override = self.get_parameter('frame_id').value
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.pixel_stride = max(1, int(self.get_parameter('pixel_stride').value))
        self.filter_by_height = bool(self.get_parameter('filter_by_height').value)
        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.camera_pitch_rad = math.radians(float(self.get_parameter('camera_pitch_deg').value))
        self.camera_offset_x = float(self.get_parameter('camera_offset_x').value)
        self.camera_offset_y = float(self.get_parameter('camera_offset_y').value)
        self.camera_offset_z = float(self.get_parameter('camera_offset_z').value)
        self.cos_pitch = math.cos(self.camera_pitch_rad)
        self.sin_pitch = math.sin(self.camera_pitch_rad)

        self.bridge = CvBridge()
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_frame = None

        self.publisher = self.create_publisher(PointCloud2, self.pointcloud_topic, 10)
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Publishing depth-derived point cloud on {self.pointcloud_topic} '
            f'from {self.depth_topic} using {self.camera_info_topic} '
            f'(height_filter={self.filter_by_height}, '
            f'height_range=[{self.min_height:.2f}, {self.max_height:.2f}] m)'
        )

    def optical_to_base_height(self, x_opt: float, y_opt: float, z_opt: float) -> float:
        # Optical frame uses x right, y down, z forward.
        # Convert to the camera frame expected by the URDF mount:
        # x forward, y left, z up.
        x_cam = z_opt
        y_cam = -x_opt
        z_cam = -y_opt

        _x_base = self.cos_pitch * x_cam + self.sin_pitch * z_cam + self.camera_offset_x
        _y_base = y_cam + self.camera_offset_y
        z_base = -self.sin_pitch * x_cam + self.cos_pitch * z_cam + self.camera_offset_z
        return z_base

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.camera_frame = msg.header.frame_id

    def depth_callback(self, msg: Image) -> None:
        if self.fx is None:
            return

        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if depth is None:
            return

        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) / 1000.0
        else:
            depth = depth.astype(np.float32)

        height, width = depth.shape[:2]
        stride = self.pixel_stride
        points = []

        for v in range(0, height, stride):
            row = depth[v]
            for u in range(0, width, stride):
                d = float(row[u])
                if not math.isfinite(d) or d < self.min_depth or d > self.max_depth:
                    continue
                x = (u - self.cx) * d / self.fx
                y = (v - self.cy) * d / self.fy
                z = d
                if self.filter_by_height:
                    z_base = self.optical_to_base_height(x, y, z)
                    if z_base < self.min_height or z_base > self.max_height:
                        continue
                points.append((x, y, z))

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.frame_id_override or self.camera_frame or msg.header.frame_id
        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.publisher.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
