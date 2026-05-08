#!/usr/bin/env python3

import math

import rclpy
from depth_to_pointcloud import DepthToPointCloud
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class PointCloudFilter(DepthToPointCloud):
    """Filter the ZED-provided PointCloud2 using the depth node's logic."""

    def __init__(self):
        Node.__init__(self, 'pointcloud_filter')

        # Keep the old depth-node parameters available so configs can be shared.
        self.declare_parameter('depth_topic', '/zed_node/left/depth_image')
        self.declare_parameter('camera_info_topic', '/zed_node/left/camera_info')
        self.declare_parameter('input_pointcloud_topic', '/zed_node/left/points')
        self.declare_parameter('pointcloud_topic', '/zed_node/left/points/filtered')
        self.declare_parameter('obstacle_pointcloud_topic', '/zed_node/left/points/obstacles')
        self.declare_parameter('ramp_pointcloud_topic', '/zed_node/left/points/ramps')
        self.declare_parameter('frame_id', 'left_camera_link_optical')
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 20.0)
        self.declare_parameter('pixel_stride', 1)
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

        self.input_pointcloud_topic = self.get_parameter('input_pointcloud_topic').value
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
        self.max_ramp_detection_distance = float(
            self.get_parameter('max_ramp_detection_distance').value
        )
        self.ramp_center_lateral_limit = max(
            0.0,
            float(self.get_parameter('ramp_center_lateral_limit').value),
        )
        self.min_ramp_points = max(1, int(self.get_parameter('min_ramp_points').value))
        self.min_ramp_slope_deg = float(self.get_parameter('min_ramp_slope_deg').value)
        self.max_ramp_slope_deg = float(self.get_parameter('max_ramp_slope_deg').value)
        self.min_ramp_forward_length = max(
            0.0,
            float(self.get_parameter('min_ramp_forward_length').value),
        )
        self.min_ramp_width = max(0.0, float(self.get_parameter('min_ramp_width').value))
        self.max_ramp_width = max(0.0, float(self.get_parameter('max_ramp_width').value))
        self.max_ramp_height = max(
            0.0,
            float(self.get_parameter('max_ramp_height').value),
        )
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
        self.ramp_grid_spike_height = max(
            0.0,
            float(self.get_parameter('ramp_grid_spike_height').value),
        )
        self.ramp_grid_min_height = float(self.get_parameter('ramp_grid_min_height').value)
        self.ramp_grid_max_height = float(self.get_parameter('ramp_grid_max_height').value)

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
            PointCloud2,
            self.input_pointcloud_topic,
            self.pointcloud_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Filtering camera PointCloud2 from {self.input_pointcloud_topic}; '
            f'publishing filtered={self.pointcloud_topic}, '
            f'obstacles={self.obstacle_pointcloud_topic}, ramps={self.ramp_pointcloud_topic}; '
            f'output_frame_id={self.frame_id_override or "input"}'
        )

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        points = []
        terrain_points = []
        for index, point in enumerate(
            point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        ):
            if index % self.pixel_stride != 0:
                continue
            x, y, z = self.read_xyz(point)
            if not self.is_valid_input_point(x, y, z):
                continue
            if self.filter_by_roi or self.filter_by_height or self.classify_ramps:
                x_base, y_base, z_base = self.optical_to_base(x, y, z)
            if self.filter_by_roi:
                if x_base < self.min_forward or x_base > self.max_forward:
                    continue
                if y_base < self.min_lateral or y_base > self.max_lateral:
                    continue
            terrain_points.append((x, y, z))
            if self.filter_by_height:
                if z_base < self.min_height or z_base > self.max_height:
                    continue
            points.append((x, y, z))

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.frame_id_override or msg.header.frame_id
        filtered_cloud = point_cloud2.create_cloud_xyz32(header, points)
        obstacle_points = self.voxel_downsample(points)
        obstacle_points = self.cluster_points(obstacle_points)
        self.ramp_points = self.detect_ramp_points_from_elevation_grid(terrain_points)
        obstacle_cloud = point_cloud2.create_cloud_xyz32(header, obstacle_points)
        ramp_cloud = point_cloud2.create_cloud_xyz32(header, self.ramp_points)

        self.publisher.publish(filtered_cloud)
        self.obstacle_publisher.publish(obstacle_cloud)
        self.ramp_publisher.publish(ramp_cloud)
        self.frame_count += 1
        self.maybe_log_cluster_metadata()

    def read_xyz(self, point) -> tuple:
        try:
            return float(point[0]), float(point[1]), float(point[2])
        except (TypeError, ValueError, IndexError):
            return float(point['x']), float(point['y']), float(point['z'])

    def is_valid_input_point(self, x: float, y: float, z: float) -> bool:
        if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
            return False
        distance = math.sqrt(x * x + y * y + z * z)
        return self.min_depth <= distance <= self.max_depth


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
