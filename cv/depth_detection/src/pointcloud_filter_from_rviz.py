#!/usr/bin/env python3

import math
import os

# numpy's OpenBLAS otherwise keeps a busy-waiting thread per core after each
# matrix operation; this node's arrays are far too small to gain from threads.
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import Pose, PoseArray  # noqa: E402
from rcl_interfaces.msg import SetParametersResult  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from scipy.sparse import coo_matrix  # noqa: E402
from scipy.sparse.csgraph import connected_components  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

EMPTY_XYZ = np.zeros((0, 3), np.float64)


def unique_rows_first(keys: np.ndarray):
    """np.unique(keys, axis=0, return_index=True, return_inverse=True) for (N, d) int64
    keys, much faster: packs each row into one int64 so numpy sorts 1-D integers."""
    lo = keys.min(axis=0)
    span = keys.max(axis=0) - lo + 1
    flat = np.zeros(len(keys), np.int64)
    for i in range(keys.shape[1]):
        flat = flat * span[i] + (keys[:, i] - lo[i])
    _, first, inverse = np.unique(flat, return_index=True, return_inverse=True)
    return first, inverse.ravel()


class PointCloudRVizFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_rviz_filter')

        # Input / output topics
        self.declare_parameter('input_topic', '/zed_node/left/points_rviz')
        self.declare_parameter('filtered_topic', '/zed_node/left/depth_points')
        self.declare_parameter('obstacle_topic', '/zed_node/left/obstacle_points')
        self.declare_parameter('ramp_topic', '/zed_node/left/ramp_points')

        # Whether to forward ramp detections to /ramp_seg (false = use this node, true = use lidar)
        self.declare_parameter('ramp_seg_using_lidar', False)

        # Shared filtering parameters (match depth_to_pointcloud)
        self.declare_parameter('filter_by_roi', True)
        self.declare_parameter('min_forward', 0.3)
        self.declare_parameter('max_forward', 6.0)
        self.declare_parameter('min_lateral', -2.0)
        self.declare_parameter('max_lateral', 2.0)
        self.declare_parameter('filter_by_height', True)
        self.declare_parameter('min_height', 0.05)
        self.declare_parameter('max_height', 2.0)
        self.declare_parameter('use_distance_height_filter', True)
        self.declare_parameter('height_filter_slope', 0.015)
        self.declare_parameter('height_filter_start_distance', 1.0)
        self.declare_parameter('height_filter_max_extra', 0.10)
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
        self.ramp_seg_using_lidar = bool(self.get_parameter('ramp_seg_using_lidar').value)

        self.filter_by_roi = bool(self.get_parameter('filter_by_roi').value)
        self.min_forward = float(self.get_parameter('min_forward').value)
        self.max_forward = float(self.get_parameter('max_forward').value)
        self.min_lateral = float(self.get_parameter('min_lateral').value)
        self.max_lateral = float(self.get_parameter('max_lateral').value)
        self.filter_by_height = bool(self.get_parameter('filter_by_height').value)
        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.use_distance_height_filter = bool(self.get_parameter('use_distance_height_filter').value)
        self.height_filter_slope = max(0.0, float(self.get_parameter('height_filter_slope').value))
        self.height_filter_start_distance = max(
            0.0,
            float(self.get_parameter('height_filter_start_distance').value),
        )
        self.height_filter_max_extra = max(0.0, float(self.get_parameter('height_filter_max_extra').value))
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
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Publishers
        self.pub_filtered = self.create_publisher(PointCloud2, self.filtered_topic, 10)
        self.pub_obstacles = self.create_publisher(PointCloud2, self.obstacle_topic, 10)
        self.pub_ramps = self.create_publisher(PointCloud2, self.ramp_topic, 10)
        self.pub_ramp_seg = self.create_publisher(PoseArray, '/ramp_seg', 10)

        # Subscription
        self.create_subscription(PointCloud2, self.input_topic, self.cloud_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'Filtering {self.input_topic} -> {self.filtered_topic},{self.obstacle_topic},{self.ramp_topic}'
            f' (ramp_seg_using_lidar={self.ramp_seg_using_lidar})'
        )

    def parameters_callback(self, params):
        for param in params:
            name = param.name
            value = param.value
            try:
                if name in (
                    'filter_by_roi',
                    'filter_by_height',
                    'voxel_downsample_obstacles',
                    'cluster_obstacles',
                    'classify_ramps',
                    'use_distance_height_filter',
                ):
                    setattr(self, name, bool(value))
                elif name == 'ramp_seg_using_lidar':
                    self.ramp_seg_using_lidar = bool(value)
                elif name in (
                    'min_forward',
                    'max_forward',
                    'min_lateral',
                    'max_lateral',
                    'min_height',
                    'max_height',
                    'camera_offset_x',
                    'camera_offset_y',
                    'camera_offset_z',
                    'height_filter_slope',
                    'max_ramp_detection_distance',
                    'ramp_grid_min_height',
                    'ramp_grid_max_height',
                    'min_ramp_slope_deg',
                    'max_ramp_slope_deg',
                ):
                    setattr(self, name, float(value))
                elif name == 'camera_pitch_deg':
                    self.camera_pitch_rad = math.radians(float(value))
                    self.cos_pitch = math.cos(self.camera_pitch_rad)
                    self.sin_pitch = math.sin(self.camera_pitch_rad)
                elif name == 'voxel_size':
                    value = float(value)
                    if value <= 0.0:
                        return SetParametersResult(successful=False, reason='voxel_size must be > 0')
                    self.voxel_size = value
                elif name == 'cluster_tolerance':
                    value = float(value)
                    if value <= 0.0:
                        return SetParametersResult(successful=False, reason='cluster_tolerance must be > 0')
                    self.cluster_tolerance = value
                elif name == 'ramp_grid_resolution':
                    value = float(value)
                    if value <= 0.0:
                        return SetParametersResult(successful=False, reason='ramp_grid_resolution must be > 0')
                    self.ramp_grid_resolution = value
                elif name in (
                    'ramp_center_lateral_limit',
                    'min_ramp_forward_length',
                    'min_ramp_width',
                    'max_ramp_width',
                    'max_ramp_height',
                    'ramp_grid_spike_height',
                    'height_filter_start_distance',
                    'height_filter_max_extra',
                ):
                    setattr(self, name, max(0.0, float(value)))
                elif name in (
                    'min_cluster_points',
                    'min_ramp_points',
                    'ramp_grid_min_points_per_cell',
                    'ramp_grid_min_connected_cells',
                ):
                    setattr(self, name, max(1, int(value)))
                elif name == 'max_cluster_points':
                    self.max_cluster_points = max(0, int(value))
            except (TypeError, ValueError) as exc:
                return SetParametersResult(successful=False, reason=f'invalid {name}: {exc}')

        return SetParametersResult(successful=True)

    # Points are (N, 3) float64 arrays (x forward, y lateral, z height) throughout,
    # vectorised with numpy; each step keeps the original per-point loop's results.

    def effective_min_height(self, x_forward):
        """Minimum obstacle height at forward distance x_forward (scalar or array)."""
        if not self.use_distance_height_filter:
            return self.min_height

        extra_height = np.maximum(
            0.0,
            x_forward - self.height_filter_start_distance,
        ) * self.height_filter_slope
        extra_height = np.minimum(extra_height, self.height_filter_max_extra)
        return self.min_height + extra_height

    def voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        """Keep the first point (in input order) of every voxel_size cube."""
        if not self.voxel_downsample_obstacles or len(points) == 0:
            return points

        first, _ = unique_rows_first(np.floor(points / self.voxel_size).astype(np.int64))
        return points[np.sort(first)]

    def cluster_points(self, points: np.ndarray) -> np.ndarray:
        """Drop points whose cluster is too small (or too big).

        Clusters are the connected components of "within cluster_tolerance of each
        other". Clusters come out in order of their first point, each cluster's
        points in input order.
        """
        if not self.cluster_obstacles or len(points) == 0:
            return points

        n = len(points)
        pairs = cKDTree(points).query_pairs(self.cluster_tolerance, output_type='ndarray')
        graph = coo_matrix((np.ones(len(pairs), np.int8), (pairs[:, 0], pairs[:, 1])), shape=(n, n))
        _, labels = connected_components(graph, directed=False)

        sizes = np.bincount(labels)
        keep = sizes >= self.min_cluster_points
        if self.max_cluster_points:
            keep &= sizes <= self.max_cluster_points
        first_index = np.full(len(sizes), n)
        np.minimum.at(first_index, labels, np.arange(n))
        idx = np.nonzero(keep[labels])[0]
        idx = idx[np.lexsort((idx, first_index[labels[idx]]))]
        return points[idx]

    def detect_ramp_points_from_elevation_grid(self, points: np.ndarray) -> np.ndarray:
        if not self.classify_ramps or len(points) == 0:
            return EMPTY_XYZ

        resolution = self.ramp_grid_resolution
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        inside = ((x >= self.min_forward) & (x <= self.max_ramp_detection_distance)
                  & (np.abs(y) <= self.ramp_center_lateral_limit)
                  & (z >= self.ramp_grid_min_height) & (z <= self.ramp_grid_max_height))
        pts = points[inside]
        if len(pts) == 0:
            return EMPTY_XYZ

        # Group into (x, y) grid cells: cells in order of their first point, each
        # cell's points in input order (the order the per-point loop built them in).
        keys = np.floor(pts[:, :2] / resolution).astype(np.int64)
        first, inverse = unique_rows_first(keys)
        by_first = np.argsort(first)
        rank = np.empty(len(first), np.int64)
        rank[by_first] = np.arange(len(first))
        point_rank = rank[inverse]
        if len(first) < np.iinfo(np.int16).max:
            point_rank = point_rank.astype(np.int16)  # numpy's stable sort is a radix sort for int16
        order = np.argsort(point_rank, kind='stable')
        counts = np.bincount(point_rank, minlength=len(first))
        cell_points = np.split(pts[order], np.cumsum(counts)[:-1])
        cell_ids = keys[first[by_first]]

        valid_cells = {}
        for (kx, ky), cell_pts in zip(cell_ids.tolist(), cell_points):
            if len(cell_pts) < self.ramp_grid_min_points_per_cell:
                continue
            point_array = cell_pts.astype(np.float32)
            height = float(np.median(point_array[:, 2]))
            valid_cells[(kx, ky)] = {
                'height': height,
                'center': (
                    float(np.mean(point_array[:, 0])),
                    float(np.mean(point_array[:, 1])),
                    height,
                ),
                'points': cell_pts,
                'point_count': len(cell_pts),
            }

        if not valid_cells:
            return EMPTY_XYZ

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
            return EMPTY_XYZ

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
                component_points.append(cell['points'])
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

        return np.concatenate(ramp_points) if ramp_points else EMPTY_XYZ

    def cloud_callback(self, msg: PointCloud2):
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        points = points[np.isfinite(points).all(axis=1)].astype(np.float64)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        # Apply ROI/height filtering directly on left_camera_link coordinates
        # (no optical_to_base transform; points_rviz frame is source of truth)
        # x: forward (X in camera frame), y: lateral, z: height
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        if self.filter_by_roi:
            in_roi = ((x >= self.min_forward) & (x <= self.max_forward)
                      & (y >= self.min_lateral) & (y <= self.max_lateral))
            terrain_points = points[in_roi]
        else:
            terrain_points = points
        filtered = terrain_points
        if self.filter_by_height:
            tz = terrain_points[:, 2]
            in_height = ((tz >= self.effective_min_height(terrain_points[:, 0]))
                         & (tz <= self.max_height))
            filtered = terrain_points[in_height]

        cloud_filtered = point_cloud2.create_cloud_xyz32(header, filtered.astype(np.float32))
        self.pub_filtered.publish(cloud_filtered)

        obstacles = self.voxel_downsample(filtered)
        obstacles = self.cluster_points(obstacles)
        cloud_obstacles = point_cloud2.create_cloud_xyz32(header, obstacles.astype(np.float32))
        self.pub_obstacles.publish(cloud_obstacles)

        ramp_points = self.detect_ramp_points_from_elevation_grid(terrain_points)
        cloud_ramps = point_cloud2.create_cloud_xyz32(header, ramp_points.astype(np.float32))
        self.pub_ramps.publish(cloud_ramps)

        if not self.ramp_seg_using_lidar and len(ramp_points):
            ramp_msg = PoseArray()
            ramp_msg.header.stamp = self.get_clock().now().to_msg()
            ramp_msg.header.frame_id = msg.header.frame_id
            for x, y, z in ramp_points.tolist():
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = z
                pose.orientation.w = 1.0
                ramp_msg.poses.append(pose)
            self.pub_ramp_seg.publish(ramp_msg)

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
