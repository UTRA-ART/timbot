#!/usr/bin/env python3

import math
import copy

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
        self.declare_parameter('classify_ramps', True)
        self.declare_parameter('max_ramp_detection_distance', 4.5)
        self.declare_parameter('ramp_center_lateral_limit', 1.2)
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

        self.classify_ramps = bool(self.get_parameter('classify_ramps').value)
        self.max_ramp_detection_distance = float(self.get_parameter('max_ramp_detection_distance').value)
        self.ramp_center_lateral_limit = float(self.get_parameter('ramp_center_lateral_limit').value)
        self.ramp_grid_min_height = float(self.get_parameter('ramp_grid_min_height').value)
        self.ramp_grid_max_height = float(self.get_parameter('ramp_grid_max_height').value)

        # Publishers
        self.pub_filtered = self.create_publisher(PointCloud2, self.filtered_topic, 10)
        self.pub_obstacles = self.create_publisher(PointCloud2, self.obstacle_topic, 10)
        self.pub_ramps = self.create_publisher(PointCloud2, self.ramp_topic, 10)

        # Subscription
        self.create_subscription(PointCloud2, self.input_topic, self.cloud_callback, qos_profile_sensor_data)

        self.get_logger().info(f'Filtering {self.input_topic} -> {self.filtered_topic},{self.obstacle_topic},{self.ramp_topic}')

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
        ramp_candidates = []
        for x_cam, y_cam, z_cam in points:
            # x_cam: forward (X in camera frame), y_cam: lateral, z_cam: height
            if self.filter_by_roi:
                if x_cam < self.min_forward or x_cam > self.max_forward:
                    continue
                if y_cam < self.min_lateral or y_cam > self.max_lateral:
                    continue
            if self.filter_by_height:
                if z_cam < self.min_height or z_cam > self.max_height:
                    continue
            filtered.append((x_cam, y_cam, z_cam))
            # Ramp candidates: same ROI/height constraints as filtered points
            if self.classify_ramps:
                if x_cam <= self.max_ramp_detection_distance and abs(y_cam) <= self.ramp_center_lateral_limit:
                    if z_cam >= self.min_height and z_cam <= self.max_height:
                        ramp_candidates.append((x_cam, y_cam, z_cam))

        # Publish filtered cloud
        cloud_filtered = point_cloud2.create_cloud_xyz32(header, filtered)
        self.pub_filtered.publish(cloud_filtered)

        # Obstacles: voxel downsample if requested
        obstacles = filtered
        if self.voxel_downsample_obstacles and self.voxel_size > 0.0 and len(filtered) > 0:
            voxels = {}
            for x, y, z in filtered:
                key = (int(math.floor(x / self.voxel_size)), int(math.floor(y / self.voxel_size)), int(math.floor(z / self.voxel_size)))
                if key not in voxels:
                    voxels[key] = (x, y, z)
            obstacles = list(voxels.values())

        cloud_obstacles = point_cloud2.create_cloud_xyz32(header, obstacles)
        self.pub_obstacles.publish(cloud_obstacles)

        # Ramps: publish simple candidate set (coarse elevation-based heuristic)
        cloud_ramps = point_cloud2.create_cloud_xyz32(header, ramp_candidates)
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
