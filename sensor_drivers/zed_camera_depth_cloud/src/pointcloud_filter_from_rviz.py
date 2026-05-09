#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class PointCloudRVizFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_rviz_filter')

        # Input / output topics
        self.declare_parameter('input_topic', '/zed_node/left/points_rviz')
        self.declare_parameter('filtered_topic', '/zed_node/left/points/filtered')
        self.declare_parameter('obstacle_topic', '/zed_node/left/points/obstacles')
        self.declare_parameter('ramp_topic', '/zed_node/left/points/ramps')

        # Shared filtering parameters (match depth_to_pointcloud)
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

        # Voxel / clustering / ramp params
        self.declare_parameter('voxel_downsample_obstacles', True)
        self.declare_parameter('voxel_size', 0.1)
        self.declare_parameter('cluster_obstacles', True)
        self.declare_parameter('cluster_tolerance', 0.35)
        self.declare_parameter('min_cluster_points', 5)
        self.declare_parameter('max_cluster_points', 0)
        self.declare_parameter('classify_ramps', True)
        self.declare_parameter('max_ramp_detection_distance', 4.5)
        self.declare_parameter('ramp_center_lateral_limit', 1.2)
        self.declare_parameter('min_ramp_points', 20)
        self.declare_parameter('min_ramp_slope_deg', 2.0)
        self.declare_parameter('max_ramp_slope_deg', 12.0)
        self.declare_parameter('min_ramp_forward_length', 1.0)
        self.declare_parameter('min_ramp_width', 0.8)
        self.declare_parameter('max_ramp_width', 3.2)
        self.declare_parameter('max_ramp_height', 1.2)
        self.declare_parameter('ramp_grid_resolution', 0.15)
        self.declare_parameter('ramp_grid_min_points_per_cell', 3)
        self.declare_parameter('ramp_grid_min_connected_cells', 8)
        self.declare_parameter('ramp_grid_spike_height', 0.25)
        self.declare_parameter('ramp_grid_min_height', -0.05)
        self.declare_parameter('ramp_grid_max_height', 1.2)

        # Read params
        self.input_topic = self.get_parameter('input_topic').value
        self.filtered_topic = self.get_parameter('filtered_topic').value
        self.obstacle_topic = self.get_parameter('obstacle_topic').value
        self.ramp_topic = self.get_parameter('ramp_topic').value

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

        self.voxel_downsample_obstacles = bool(self.get_parameter('voxel_downsample_obstacles').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.cluster_obstacles = bool(self.get_parameter('cluster_obstacles').value)
        self.cluster_tolerance = float(self.get_parameter('cluster_tolerance').value)
        self.min_cluster_points = max(1, int(self.get_parameter('min_cluster_points').value))
        self.max_cluster_points = max(0, int(self.get_parameter('max_cluster_points').value))

        self.classify_ramps = bool(self.get_parameter('classify_ramps').value)
        self.max_ramp_detection_distance = float(self.get_parameter('max_ramp_detection_distance').value)
        self.ramp_center_lateral_limit = max(0.0, float(self.get_parameter('ramp_center_lateral_limit').value))
        self.min_ramp_points = max(1, int(self.get_parameter('min_ramp_points').value))
        self.min_ramp_slope_deg = float(self.get_parameter('min_ramp_slope_deg').value)
        self.max_ramp_slope_deg = float(self.get_parameter('max_ramp_slope_deg').value)
        self.min_ramp_forward_length = max(0.0, float(self.get_parameter('min_ramp_forward_length').value))
        self.min_ramp_width = max(0.0, float(self.get_parameter('min_ramp_width').value))
        self.max_ramp_width = max(0.0, float(self.get_parameter('max_ramp_width').value))
        self.max_ramp_height = max(0.0, float(self.get_parameter('max_ramp_height').value))
        self.ramp_grid_resolution = float(self.get_parameter('ramp_grid_resolution').value)
        if self.ramp_grid_resolution <= 0.0:
            self.get_logger().warn(
                f'Invalid ramp_grid_resolution={self.ramp_grid_resolution:.3f}; '
                'disabling ramp classification'
            )
            self.classify_ramps = False
        self.ramp_grid_min_points_per_cell = max(
            1,
            int(self.get_parameter('ramp_grid_min_points_per_cell').value),
        )
        self.ramp_grid_min_connected_cells = max(
            1,
            int(self.get_parameter('ramp_grid_min_connected_cells').value),
        )
        self.ramp_grid_spike_height = max(0.0, float(self.get_parameter('ramp_grid_spike_height').value))
        self.ramp_grid_min_height = float(self.get_parameter('ramp_grid_min_height').value)
        self.ramp_grid_max_height = float(self.get_parameter('ramp_grid_max_height').value)

        # Publishers
        self.pub_filtered = self.create_publisher(PointCloud2, self.filtered_topic, 10)
        self.pub_obstacles = self.create_publisher(PointCloud2, self.obstacle_topic, 10)
        self.pub_ramps = self.create_publisher(PointCloud2, self.ramp_topic, 10)

        # Subscription
        self.create_subscription(PointCloud2, self.input_topic, self.cloud_callback, qos_profile_sensor_data)

        self.get_logger().info(f'Filtering {self.input_topic} -> {self.filtered_topic},{self.obstacle_topic},{self.ramp_topic}')

    def voxel_downsample(self, points: list) -> list:
        if not self.voxel_downsample_obstacles or not points:
            return points

        voxels = {}
        for x, y, z in points:
            key = (
                math.floor(x / self.voxel_size),
                math.floor(y / self.voxel_size),
                math.floor(z / self.voxel_size),
            )
            if key not in voxels:
                voxels[key] = (x, y, z)
        return list(voxels.values())

    def cluster_points(self, points: list) -> list:
        if not self.cluster_obstacles or not points:
            return points

        tolerance_sq = self.cluster_tolerance * self.cluster_tolerance
        cell_size = self.cluster_tolerance
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

    def detect_ramp_points_from_elevation_grid(self, points: list) -> list:
        if not self.classify_ramps or not points:
            return []

        resolution = self.ramp_grid_resolution
        grid = {}
        for x, y, z in points:
            if x < self.min_forward or x > self.max_ramp_detection_distance:
                continue
            if abs(y) > self.ramp_center_lateral_limit:
                continue
            if z < self.ramp_grid_min_height or z > self.ramp_grid_max_height:
                continue

            key = (math.floor(x / resolution), math.floor(y / resolution))
            cell = grid.setdefault(
                key,
                {
                    'points': [],
                    'heights': [],
                },
            )
            cell['points'].append((x, y, z))
            cell['heights'].append(z)

        valid_cells = {}
        for key, cell in grid.items():
            if len(cell['heights']) < self.ramp_grid_min_points_per_cell:
                continue
            point_array = np.asarray(cell['points'], dtype=np.float32)
            height = float(np.median(np.asarray(cell['heights'], dtype=np.float32)))
            valid_cells[key] = {
                'height': height,
                'center': (
                    float(np.mean(point_array[:, 0])),
                    float(np.mean(point_array[:, 1])),
                    height,
                ),
                'points': cell['points'],
                'point_count': len(cell['points']),
            }

        if not valid_cells:
            return []

        filtered_cells = {}
        for key, cell in valid_cells.items():
            neighbor_heights = []
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    neighbor = valid_cells.get((key[0] + dx, key[1] + dy))
                    if neighbor is not None:
                        neighbor_heights.append(neighbor['height'])

            if not neighbor_heights:
                continue
            neighbor_median = float(np.median(np.asarray(neighbor_heights, dtype=np.float32)))
            if cell['height'] - neighbor_median > self.ramp_grid_spike_height:
                continue
            filtered_cells[key] = cell

        slope_cells = {}
        for key, cell in filtered_cells.items():
            best_slope_deg = None
            for forward_step in (1, 2):
                for dy in (-1, 0, 1):
                    forward_neighbor = filtered_cells.get((key[0] + forward_step, key[1] + dy))
                    if forward_neighbor is None:
                        continue
                    dx = forward_neighbor['center'][0] - cell['center'][0]
                    if dx <= 1e-6:
                        continue
                    dz = forward_neighbor['height'] - cell['height']
                    slope_deg = math.degrees(math.atan2(dz, dx))
                    if self.min_ramp_slope_deg <= slope_deg <= self.max_ramp_slope_deg:
                        if best_slope_deg is None or slope_deg > best_slope_deg:
                            best_slope_deg = slope_deg

            if best_slope_deg is not None:
                slope_cells[key] = {
                    **cell,
                    'slope_deg': best_slope_deg,
                }

        if not slope_cells:
            return []

        visited = set()
        ramp_points = []
        for start_key in slope_cells:
            if start_key in visited:
                continue

            queue = [start_key]
            visited.add(start_key)
            component_keys = []
            while queue:
                current_key = queue.pop()
                component_keys.append(current_key)
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        neighbor_key = (current_key[0] + dx, current_key[1] + dy)
                        if neighbor_key in visited or neighbor_key not in slope_cells:
                            continue
                        visited.add(neighbor_key)
                        queue.append(neighbor_key)

            if len(component_keys) < self.ramp_grid_min_connected_cells:
                continue

            component_points = []
            forward_values = []
            lateral_values = []
            height_values = []
            point_count = 0
            for key in component_keys:
                cell = slope_cells[key]
                component_points.extend(cell['points'])
                point_count += cell['point_count']
                center = cell['center']
                forward_values.append(center[0])
                lateral_values.append(center[1])
                height_values.append(center[2])

            if point_count < self.min_ramp_points:
                continue

            min_forward = min(forward_values)
            max_forward = max(forward_values)
            min_lateral = min(lateral_values)
            max_lateral = max(lateral_values)
            min_height = min(height_values)
            max_height = max(height_values)
            length = max_forward - min_forward
            width = max_lateral - min_lateral + resolution
            height = max_height - min_height
            center_lateral = 0.5 * (min_lateral + max_lateral)

            if min_forward > self.max_ramp_detection_distance:
                continue
            if abs(center_lateral) > self.ramp_center_lateral_limit:
                continue
            if length < self.min_ramp_forward_length:
                continue
            if width < self.min_ramp_width:
                continue
            if self.max_ramp_width and width > self.max_ramp_width:
                continue
            if height > self.max_ramp_height:
                continue

            ramp_points.extend(component_points)

        return ramp_points

    def cloud_callback(self, msg: PointCloud2):
        # Read points
        points_iter = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = []
        for p in points_iter:
            x, y, z = p
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            points.append((float(x), float(y), float(z)))

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        # Apply ROI/height filtering directly on left_camera_link coordinates
        # (no optical_to_base transform; points_rviz frame is source of truth)
        filtered = []
        terrain_points = []
        for x_cam, y_cam, z_cam in points:
            # x_cam: forward (X in camera frame), y_cam: lateral, z_cam: height
            if self.filter_by_roi:
                if x_cam < self.min_forward or x_cam > self.max_forward:
                    continue
                if y_cam < self.min_lateral or y_cam > self.max_lateral:
                    continue
            terrain_points.append((x_cam, y_cam, z_cam))
            if self.filter_by_height:
                if z_cam < self.min_height or z_cam > self.max_height:
                    continue
            filtered.append((x_cam, y_cam, z_cam))

        # Publish filtered cloud
        cloud_filtered = point_cloud2.create_cloud_xyz32(header, filtered)
        self.pub_filtered.publish(cloud_filtered)

        obstacles = self.voxel_downsample(filtered)
        obstacles = self.cluster_points(obstacles)
        cloud_obstacles = point_cloud2.create_cloud_xyz32(header, obstacles)
        self.pub_obstacles.publish(cloud_obstacles)

        ramp_points = self.detect_ramp_points_from_elevation_grid(terrain_points)
        cloud_ramps = point_cloud2.create_cloud_xyz32(header, ramp_points)
        self.pub_ramps.publish(cloud_ramps)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudRVizFilter()
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
