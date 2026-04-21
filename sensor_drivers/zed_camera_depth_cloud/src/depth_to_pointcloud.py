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
        self.declare_parameter('obstacle_pointcloud_topic', '/zed_node/left/obstacle_points')
        self.declare_parameter('frame_id', 'left_camera_link_optical')
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 20.0)
        self.declare_parameter('pixel_stride', 2)
        self.declare_parameter('filter_by_roi', True)
        self.declare_parameter('min_forward', 0.3)
        self.declare_parameter('max_forward', 6.0)
        self.declare_parameter('min_lateral', -2.0)
        self.declare_parameter('max_lateral', 2.0)
        self.declare_parameter('filter_by_height', True)
        self.declare_parameter('min_height', 0.05)
        self.declare_parameter('max_height', 2.0)
        self.declare_parameter('camera_pitch_deg', 25.0)
        self.declare_parameter('camera_offset_x', 0.222173)
        self.declare_parameter('camera_offset_y', 0.061524)
        self.declare_parameter('camera_offset_z', 0.71)
        self.declare_parameter('voxel_downsample_obstacles', True)
        self.declare_parameter('voxel_size', 0.1)
        self.declare_parameter('cluster_obstacles', True)
        self.declare_parameter('cluster_tolerance', 0.35)
        self.declare_parameter('min_cluster_points', 5)
        self.declare_parameter('max_cluster_points', 0)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.obstacle_pointcloud_topic = self.get_parameter('obstacle_pointcloud_topic').value
        self.frame_id_override = self.get_parameter('frame_id').value
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.pixel_stride = max(1, int(self.get_parameter('pixel_stride').value))
        self.filter_by_roi = bool(self.get_parameter('filter_by_roi').value)
        self.min_forward = float(self.get_parameter('min_forward').value)
        self.max_forward = float(self.get_parameter('max_forward').value)
        self.min_lateral = float(self.get_parameter('min_lateral').value)
        self.max_lateral = float(self.get_parameter('max_lateral').value)
        self.filter_by_height = bool(self.get_parameter('filter_by_height').value)
        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.camera_pitch_rad = math.radians(float(self.get_parameter('camera_pitch_deg').value))
        self.camera_offset_x = float(self.get_parameter('camera_offset_x').value)
        self.camera_offset_y = float(self.get_parameter('camera_offset_y').value)
        self.camera_offset_z = float(self.get_parameter('camera_offset_z').value)
        self.cos_pitch = math.cos(self.camera_pitch_rad)
        self.sin_pitch = math.sin(self.camera_pitch_rad)
        self.voxel_downsample_obstacles = bool(
            self.get_parameter('voxel_downsample_obstacles').value
        )
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        if self.voxel_size <= 0.0:
            self.get_logger().warn(
                f'Invalid voxel_size={self.voxel_size:.3f}; disabling obstacle downsampling'
            )
            self.voxel_downsample_obstacles = False
        self.cluster_obstacles = bool(self.get_parameter('cluster_obstacles').value)
        self.cluster_tolerance = float(self.get_parameter('cluster_tolerance').value)
        self.min_cluster_points = max(1, int(self.get_parameter('min_cluster_points').value))
        self.max_cluster_points = max(0, int(self.get_parameter('max_cluster_points').value))
        if self.cluster_tolerance <= 0.0:
            self.get_logger().warn(
                f'Invalid cluster_tolerance={self.cluster_tolerance:.3f}; disabling clustering'
            )
            self.cluster_obstacles = False

        self.bridge = CvBridge()
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_frame = None

        self.publisher = self.create_publisher(PointCloud2, self.pointcloud_topic, 10)
        self.obstacle_publisher = self.create_publisher(
            PointCloud2,
            self.obstacle_pointcloud_topic,
            10,
        )
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
            f'and obstacle point cloud on {self.obstacle_pointcloud_topic} '
            f'from {self.depth_topic} using {self.camera_info_topic} '
            f'(roi_filter={self.filter_by_roi}, '
            f'forward_range=[{self.min_forward:.2f}, {self.max_forward:.2f}] m, '
            f'lateral_range=[{self.min_lateral:.2f}, {self.max_lateral:.2f}] m, '
            f'height_filter={self.filter_by_height}, '
            f'height_range=[{self.min_height:.2f}, {self.max_height:.2f}] m, '
            f'obstacle_voxel_downsample={self.voxel_downsample_obstacles}, '
            f'voxel_size={self.voxel_size:.2f} m, '
            f'cluster_obstacles={self.cluster_obstacles}, '
            f'cluster_tolerance={self.cluster_tolerance:.2f} m, '
            f'min_cluster_points={self.min_cluster_points}, '
            f'max_cluster_points={self.max_cluster_points})'
        )

    def voxel_downsample(self, points: list) -> list:
        if not self.voxel_downsample_obstacles or not points:
            return points

        voxels = {}
        voxel_size = self.voxel_size
        for x, y, z in points:
            key = (
                math.floor(x / voxel_size),
                math.floor(y / voxel_size),
                math.floor(z / voxel_size),
            )
            if key not in voxels:
                voxels[key] = (x, y, z)

        return list(voxels.values())

    def cluster_points(self, points: list) -> list:
        if not self.cluster_obstacles or not points:
            return points

        tolerance = self.cluster_tolerance
        tolerance_sq = tolerance * tolerance
        cell_size = tolerance
        spatial_grid = {}

        for index, (x, y, z) in enumerate(points):
            key = (
                math.floor(x / cell_size),
                math.floor(y / cell_size),
                math.floor(z / cell_size),
            )
            spatial_grid.setdefault(key, []).append(index)

        visited = [False] * len(points)
        clustered_points = []
        neighbor_offsets = [
            (dx, dy, dz)
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            for dz in (-1, 0, 1)
        ]

        for start_index in range(len(points)):
            if visited[start_index]:
                continue

            visited[start_index] = True
            queue = [start_index]
            cluster_indices = []

            while queue:
                current_index = queue.pop()
                cluster_indices.append(current_index)
                x, y, z = points[current_index]
                current_key = (
                    math.floor(x / cell_size),
                    math.floor(y / cell_size),
                    math.floor(z / cell_size),
                )

                for dx, dy, dz in neighbor_offsets:
                    neighbor_key = (
                        current_key[0] + dx,
                        current_key[1] + dy,
                        current_key[2] + dz,
                    )
                    for neighbor_index in spatial_grid.get(neighbor_key, []):
                        if visited[neighbor_index]:
                            continue
                        nx, ny, nz = points[neighbor_index]
                        distance_sq = (
                            (x - nx) * (x - nx)
                            + (y - ny) * (y - ny)
                            + (z - nz) * (z - nz)
                        )
                        if distance_sq <= tolerance_sq:
                            visited[neighbor_index] = True
                            queue.append(neighbor_index)

            cluster_size = len(cluster_indices)
            if cluster_size < self.min_cluster_points:
                continue
            if self.max_cluster_points and cluster_size > self.max_cluster_points:
                continue
            clustered_points.extend(points[index] for index in cluster_indices)

        return clustered_points

    def optical_to_base(self, x_opt: float, y_opt: float, z_opt: float) -> tuple:
        # Optical frame uses x right, y down, z forward.
        # Convert to the camera frame expected by the URDF mount:
        # x forward, y left, z up.
        x_cam = z_opt
        y_cam = -x_opt
        z_cam = -y_opt

        x_base = self.cos_pitch * x_cam + self.sin_pitch * z_cam + self.camera_offset_x
        y_base = y_cam + self.camera_offset_y
        z_base = -self.sin_pitch * x_cam + self.cos_pitch * z_cam + self.camera_offset_z
        return x_base, y_base, z_base

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
                if self.filter_by_roi or self.filter_by_height:
                    x_base, y_base, z_base = self.optical_to_base(x, y, z)
                if self.filter_by_roi:
                    if x_base < self.min_forward or x_base > self.max_forward:
                        continue
                    if y_base < self.min_lateral or y_base > self.max_lateral:
                        continue
                if self.filter_by_height:
                    if z_base < self.min_height or z_base > self.max_height:
                        continue
                points.append((x, y, z))

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.frame_id_override or self.camera_frame or msg.header.frame_id
        cloud = point_cloud2.create_cloud_xyz32(header, points)
        obstacle_points = self.voxel_downsample(points)
        obstacle_points = self.cluster_points(obstacle_points)
        obstacle_cloud = point_cloud2.create_cloud_xyz32(header, obstacle_points)
        self.publisher.publish(cloud)
        self.obstacle_publisher.publish(obstacle_cloud)


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
