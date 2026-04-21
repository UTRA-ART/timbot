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
        self.declare_parameter('ramp_pointcloud_topic', '/zed_node/left/ramp_points')
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
        self.declare_parameter('log_cluster_metadata', False)
        self.declare_parameter('cluster_metadata_log_period', 30)
        self.declare_parameter('classify_ramps', True)
        self.declare_parameter('ramp_hypotenuse', 6.0)
        self.declare_parameter('ramp_hypotenuse_tolerance', 1.5)
        self.declare_parameter('ramp_length', 5.9)
        self.declare_parameter('ramp_length_tolerance', 1.5)
        self.declare_parameter('ramp_width', 2.41)
        self.declare_parameter('ramp_width_tolerance', 0.8)
        self.declare_parameter('ramp_height', 0.67)
        self.declare_parameter('ramp_height_tolerance', 0.35)
        self.declare_parameter('ramp_incline_deg', 6.43)
        self.declare_parameter('ramp_incline_tolerance_deg', 5.0)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.obstacle_pointcloud_topic = self.get_parameter('obstacle_pointcloud_topic').value
        self.ramp_pointcloud_topic = self.get_parameter('ramp_pointcloud_topic').value
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
        self.log_cluster_metadata = bool(self.get_parameter('log_cluster_metadata').value)
        self.cluster_metadata_log_period = max(
            1,
            int(self.get_parameter('cluster_metadata_log_period').value),
        )
        self.classify_ramps = bool(self.get_parameter('classify_ramps').value)
        self.ramp_hypotenuse = float(self.get_parameter('ramp_hypotenuse').value)
        self.ramp_hypotenuse_tolerance = max(
            0.0,
            float(self.get_parameter('ramp_hypotenuse_tolerance').value),
        )
        self.ramp_length = float(self.get_parameter('ramp_length').value)
        self.ramp_length_tolerance = max(
            0.0,
            float(self.get_parameter('ramp_length_tolerance').value),
        )
        self.ramp_width = float(self.get_parameter('ramp_width').value)
        self.ramp_width_tolerance = max(
            0.0,
            float(self.get_parameter('ramp_width_tolerance').value),
        )
        self.ramp_height = float(self.get_parameter('ramp_height').value)
        self.ramp_height_tolerance = max(
            0.0,
            float(self.get_parameter('ramp_height_tolerance').value),
        )
        self.ramp_incline_deg = float(self.get_parameter('ramp_incline_deg').value)
        self.ramp_incline_tolerance_deg = max(
            0.0,
            float(self.get_parameter('ramp_incline_tolerance_deg').value),
        )

        self.bridge = CvBridge()
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_frame = None
        self.frame_count = 0
        self.cluster_metadata = []
        self.ramp_metadata = []
        self.ramp_points = []

        self.publisher = self.create_publisher(PointCloud2, self.pointcloud_topic, 10)
        self.obstacle_publisher = self.create_publisher(
            PointCloud2,
            self.obstacle_pointcloud_topic,
            10,
        )
        self.ramp_publisher = self.create_publisher(
            PointCloud2,
            self.ramp_pointcloud_topic,
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
            f'and ramp point cloud on {self.ramp_pointcloud_topic} '
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
            f'max_cluster_points={self.max_cluster_points}, '
            f'log_cluster_metadata={self.log_cluster_metadata}, '
            f'classify_ramps={self.classify_ramps}, '
            f'ramp_hypotenuse={self.ramp_hypotenuse:.2f} m, '
            f'ramp_size=({self.ramp_length:.2f}, {self.ramp_width:.2f}, {self.ramp_height:.2f}) m, '
            f'ramp_incline={self.ramp_incline_deg:.2f} deg)'
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
            self.cluster_metadata = []
            self.ramp_metadata = []
            self.ramp_points = []
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
        cluster_metadata = []
        ramp_points = []
        ramp_metadata = []
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

            cluster_points = [points[index] for index in cluster_indices]
            cluster_id = len(cluster_metadata) + len(ramp_metadata) + 1
            metadata = self.compute_cluster_metadata(cluster_id, cluster_points)
            if self.is_ramp_cluster(metadata):
                metadata['classification'] = 'ramp'
                ramp_metadata.append(metadata)
                ramp_points.extend(cluster_points)
            else:
                metadata['classification'] = 'obstacle'
                cluster_metadata.append(metadata)
                clustered_points.extend(cluster_points)

        self.cluster_metadata = cluster_metadata
        self.ramp_metadata = ramp_metadata
        self.ramp_points = ramp_points
        return clustered_points

    def compute_cluster_metadata(self, cluster_id: int, cluster_points: list) -> dict:
        base_points = [self.optical_to_base(x, y, z) for x, y, z in cluster_points]
        forward_values = [point[0] for point in base_points]
        lateral_values = [point[1] for point in base_points]
        height_values = [point[2] for point in base_points]

        min_forward = min(forward_values)
        max_forward = max(forward_values)
        min_lateral = min(lateral_values)
        max_lateral = max(lateral_values)
        min_height = min(height_values)
        max_height = max(height_values)

        center_forward = 0.5 * (min_forward + max_forward)
        center_lateral = 0.5 * (min_lateral + max_lateral)
        center_height = 0.5 * (min_height + max_height)
        length = max_forward - min_forward
        width = max_lateral - min_lateral
        height = max_height - min_height
        hypotenuse = math.hypot(length, height)
        distances = [math.hypot(point[0], point[1]) for point in base_points]
        slope = self.estimate_cluster_slope(forward_values, height_values)

        return {
            'id': cluster_id,
            'point_count': len(cluster_points),
            'center': (center_forward, center_lateral, center_height),
            'size': (length, width, height),
            'hypotenuse': hypotenuse,
            'forward_range': (min_forward, max_forward),
            'lateral_range': (min_lateral, max_lateral),
            'height_range': (min_height, max_height),
            'closest_distance': min(distances),
            'farthest_distance': max(distances),
            'slope': slope,
            'slope_deg': math.degrees(math.atan(slope)),
        }

    def estimate_cluster_slope(self, forward_values: list, height_values: list) -> float:
        if len(forward_values) < 2:
            return 0.0

        mean_forward = sum(forward_values) / len(forward_values)
        mean_height = sum(height_values) / len(height_values)
        variance_forward = sum(
            (forward - mean_forward) * (forward - mean_forward)
            for forward in forward_values
        )
        if variance_forward <= 1e-6:
            return 0.0

        covariance = sum(
            (forward - mean_forward) * (height - mean_height)
            for forward, height in zip(forward_values, height_values)
        )
        return covariance / variance_forward

    def is_ramp_cluster(self, metadata: dict) -> bool:
        if not self.classify_ramps:
            return False

        length, width, height = metadata['size']
        incline_deg = abs(metadata['slope_deg'])

        hypotenuse = metadata['hypotenuse']

        hypotenuse_matches = (
            abs(hypotenuse - self.ramp_hypotenuse) <= self.ramp_hypotenuse_tolerance
        )
        length_matches = abs(length - self.ramp_length) <= self.ramp_length_tolerance
        width_matches = abs(width - self.ramp_width) <= self.ramp_width_tolerance
        height_matches = abs(height - self.ramp_height) <= self.ramp_height_tolerance
        incline_matches = (
            abs(incline_deg - self.ramp_incline_deg) <= self.ramp_incline_tolerance_deg
        )

        return (
            hypotenuse_matches
            and length_matches
            and width_matches
            and height_matches
            and incline_matches
        )

    def maybe_log_cluster_metadata(self) -> None:
        if not self.log_cluster_metadata:
            return
        if self.frame_count % self.cluster_metadata_log_period != 0:
            return
        if not self.cluster_metadata and not self.ramp_metadata:
            self.get_logger().info('Cluster metadata: no accepted obstacle or ramp clusters')
            return

        summaries = []
        for metadata in self.cluster_metadata[:5]:
            center = metadata['center']
            size = metadata['size']
            forward_range = metadata['forward_range']
            height_range = metadata['height_range']
            summaries.append(
                f"obstacle id={metadata['id']} points={metadata['point_count']} "
                f"center=({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}) "
                f"size=({size[0]:.2f}, {size[1]:.2f}, {size[2]:.2f}) "
                f"hyp={metadata['hypotenuse']:.2f} "
                f"forward=({forward_range[0]:.2f}, {forward_range[1]:.2f}) "
                f"height=({height_range[0]:.2f}, {height_range[1]:.2f}) "
                f"slope={metadata['slope_deg']:.1f}deg"
            )
        for metadata in self.ramp_metadata[:5]:
            center = metadata['center']
            size = metadata['size']
            forward_range = metadata['forward_range']
            height_range = metadata['height_range']
            summaries.append(
                f"ramp id={metadata['id']} points={metadata['point_count']} "
                f"center=({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}) "
                f"size=({size[0]:.2f}, {size[1]:.2f}, {size[2]:.2f}) "
                f"hyp={metadata['hypotenuse']:.2f} "
                f"forward=({forward_range[0]:.2f}, {forward_range[1]:.2f}) "
                f"height=({height_range[0]:.2f}, {height_range[1]:.2f}) "
                f"slope={metadata['slope_deg']:.1f}deg"
            )

        extra = ''
        total_metadata = len(self.cluster_metadata) + len(self.ramp_metadata)
        if total_metadata > len(summaries):
            extra = f"; +{total_metadata - len(summaries)} more"
        self.get_logger().info(
            f"Cluster metadata ({len(self.cluster_metadata)} obstacles, "
            f"{len(self.ramp_metadata)} ramps): "
            + '; '.join(summaries)
            + extra
        )

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
        ramp_cloud = point_cloud2.create_cloud_xyz32(header, self.ramp_points)
        self.publisher.publish(cloud)
        self.obstacle_publisher.publish(obstacle_cloud)
        self.ramp_publisher.publish(ramp_cloud)
        self.frame_count += 1
        self.maybe_log_cluster_metadata()


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
