#!/usr/bin/env python3
"""
Dual LiDAR Filter Node

Compares readings from two LiDAR sensors (upper and lower) to detect ramps.
Ramps appear as depth differences between the sensors.

Publishes:
    /scan_modified - Filtered LiDAR with ramp points removed
    /ramp_seg - PoseArray of detected ramp segment points

Subscribes:
    /scan_lower - Main (lower) LiDAR
    /scan_upper or /scan_lower - Upper LiDAR (can be same as lower in sim)
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math

from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header

import tf2_ros
from tf2_ros import TransformException
from laser_geometry import LaserProjection
import sensor_msgs_py.point_cloud2 as pc2


def get_expected_ramp_depth(theta_degrees: float, lidar_distance: float) -> float:
    """Calculate expected depth difference for ramp detection."""
    return lidar_distance / math.tan(math.radians(theta_degrees))


class DualLidarFilterNode(Node):
    def __init__(self):
        super().__init__('dual_lidar_filter')

        # Declare parameters with defaults
        self.declare_parameter('main_lidar_topic', '/scan_lower')
        self.declare_parameter('upper_lidar_topic', '/scan_lower')
        self.declare_parameter('out_topic', '/scan_modified')
        self.declare_parameter('distance_to_second_lidar', 0.14)
        self.declare_parameter('max_theta_degrees', 20.0)
        self.declare_parameter('compare_lidar_time_tolerance_seconds', 2)
        self.declare_parameter('upper_lidar_angular_total_range', 360)
        self.declare_parameter('upper_lidar_start_index', 0)
        self.declare_parameter('upper_lidar_stop_index', 1080)
        self.declare_parameter('main_lidar_angular_total_range', 360)
        self.declare_parameter('limit_output_range', True)
        self.declare_parameter('desired_output_total_range', 180)

        # Get parameters
        self.main_lidar_topic = self.get_parameter('main_lidar_topic').value
        self.upper_lidar_topic = self.get_parameter('upper_lidar_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.lidar_dist = self.get_parameter('distance_to_second_lidar').value
        self.max_theta_degrees = self.get_parameter('max_theta_degrees').value
        self.comp_lidar_tol_secs = self.get_parameter('compare_lidar_time_tolerance_seconds').value
        self.upper_lidar_angular_range = self.get_parameter('upper_lidar_angular_total_range').value
        self.upper_lidar_start_index = self.get_parameter('upper_lidar_start_index').value
        self.upper_lidar_stop_index = self.get_parameter('upper_lidar_stop_index').value
        self.main_lidar_angular_range = self.get_parameter('main_lidar_angular_total_range').value
        self.limit_output_range = self.get_parameter('limit_output_range').value
        self.desired_output_range = self.get_parameter('desired_output_total_range').value

        # Validate desired_output_range
        if self.limit_output_range and self.desired_output_range > self.main_lidar_angular_range:
            self.get_logger().warn('Desired lidar range is larger than possible lidar range. Ignoring this filtering.')
            self.limit_output_range = False

        # Calculate ramp depth threshold
        self.min_ramp_depth = get_expected_ramp_depth(self.max_theta_degrees, self.lidar_dist)
        self.second_len = self.upper_lidar_stop_index - self.upper_lidar_start_index

        self.get_logger().info(f'Min ramp depth threshold: {self.min_ramp_depth:.3f}m')

        # State
        self.last_upper_ranges = []
        self.last_upper_stamp = 0
        self.init_lidar_fill = 0

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.laser_projector = LaserProjection()

        # Publishers
        self.out_pub = self.create_publisher(LaserScan, self.out_topic, 10)
        self.ramp_seg_pub = self.create_publisher(PoseArray, '/ramp_seg', 10)

        # Subscribers
        self.main_lidar_sub = self.create_subscription(
            LaserScan, self.main_lidar_topic, self.lidar_callback, 10
        )
        self.upper_lidar_sub = self.create_subscription(
            LaserScan, self.upper_lidar_topic, self.upper_lidar_callback, 10
        )

        self.get_logger().info(f'DualLidarFilterNode started')
        self.get_logger().info(f'  Main LiDAR: {self.main_lidar_topic}')
        self.get_logger().info(f'  Upper LiDAR: {self.upper_lidar_topic}')
        self.get_logger().info(f'  Output: {self.out_topic}')

    def get_second_idx_from_first_idx(self, prim_idx: int, main_lidar_len: int) -> int:
        """Map index from main lidar to corresponding upper lidar index."""
        ang_diff = (prim_idx / main_lidar_len - 0.5) * self.main_lidar_angular_range
        second_i_diff = ang_diff / self.upper_lidar_angular_range
        second_i_delt = (second_i_diff + 0.5) * self.second_len
        second_i = round(second_i_delt + self.upper_lidar_start_index)
        return min(max(second_i, 0), len(self.last_upper_ranges) - 1)

    def upper_lidar_callback(self, msg: LaserScan):
        """Store upper lidar data for comparison."""
        self.last_upper_ranges = list(msg.ranges)
        self.last_upper_stamp = msg.header.stamp.sec

    def lidar_callback(self, msg: LaserScan):
        """Process main lidar, filter ramps, publish results."""
        # Check if we have upper lidar data and it's recent enough
        delt_secs = abs(msg.header.stamp.sec - self.last_upper_stamp)
        has_valid_upper = len(self.last_upper_ranges) >= self.second_len
        acceptable_time_diff = delt_secs < self.comp_lidar_tol_secs

        if has_valid_upper and acceptable_time_diff:
            # Create output message
            out_msg = LaserScan()
            out_msg.header = msg.header
            out_msg.angle_min = msg.angle_min
            out_msg.angle_max = msg.angle_max
            out_msg.angle_increment = msg.angle_increment
            out_msg.time_increment = msg.time_increment
            out_msg.scan_time = msg.scan_time
            out_msg.range_min = msg.range_min
            out_msg.range_max = msg.range_max

            # Filter ranges
            out_msg.ranges = self.ramp_filter(list(msg.ranges))
            out_msg.intensities = list(msg.intensities) if msg.intensities else []

            self.out_pub.publish(out_msg)

            # Detect and publish ramp segments
            self.detect_and_publish_ramp(msg)
        else:
            # No valid upper lidar data, pass through original
            self.out_pub.publish(msg)

    def ramp_filter(self, main_ranges: list) -> list:
        """Filter out ramp points by comparing with upper lidar."""
        out = main_ranges.copy()
        size = len(out)
        all_inf = True

        # Calculate range limiting indices
        angle_to_idx = size / self.main_lidar_angular_range
        begin_idx = int((self.main_lidar_angular_range - self.desired_output_range) * angle_to_idx / 2)
        end_idx = int(size - begin_idx)

        for i in range(size):
            upper_idx = self.get_second_idx_from_first_idx(i, size)
            if upper_idx < len(self.last_upper_ranges):
                comp_depth = self.last_upper_ranges[upper_idx]
                depth = comp_depth - main_ranges[i]

                if depth > self.min_ramp_depth:
                    out[i] = float('inf')  # Remove ramp points
                else:
                    out[i] = main_ranges[i]
            else:
                out[i] = main_ranges[i]

            # Apply range limiting
            if self.limit_output_range and (i < begin_idx or i > end_idx):
                out[i] = float('nan')
            elif not math.isinf(out[i]) and not math.isnan(out[i]):
                all_inf = False

        # Cartographer fix: if all inf, set one point to avoid issues
        if all_inf:
            out = [float('nan')] * size
            if self.init_lidar_fill < 50:
                out[0] = 3.0
                self.init_lidar_fill += 1

        return out

    def get_all_deeper_segments(self, main_ranges: list) -> list:
        """Find all contiguous segments where ramp is detected."""
        segments = []
        in_ramp = False
        size = len(main_ranges)

        for i in range(size):
            upper_idx = self.get_second_idx_from_first_idx(i, size)
            if upper_idx < len(self.last_upper_ranges):
                comp_depth = self.last_upper_ranges[upper_idx]
                depth = comp_depth - main_ranges[i]

                if depth > self.min_ramp_depth:
                    if not in_ramp:
                        segments.append([])
                    in_ramp = True
                    segments[-1].append(i)
                else:
                    in_ramp = False

        return segments

    def detect_and_publish_ramp(self, msg: LaserScan):
        """Detect ramp segments and publish to /ramp_seg."""
        try:
            # Convert laser scan to point cloud for coordinates
            cloud = self.laser_projector.projectLaser(msg)

            # Get ramp segments
            segments = self.get_all_deeper_segments(list(msg.ranges))

            if not segments:
                return

            # Find largest segment
            largest_segment = max(segments, key=len)

            if len(largest_segment) < 2:
                return

            # Create PoseArray message
            ramp_msg = PoseArray()
            ramp_msg.header.stamp = self.get_clock().now().to_msg()
            ramp_msg.header.frame_id = msg.header.frame_id

            # Extract points from cloud
            points = list(pc2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True))

            for idx in largest_segment:
                if idx < len(points):
                    pose = Pose()
                    pose.position.x = points[idx][0]
                    pose.position.y = points[idx][1]
                    pose.position.z = points[idx][2]
                    pose.orientation.w = 1.0
                    ramp_msg.poses.append(pose)

            if ramp_msg.poses:
                self.ramp_seg_pub.publish(ramp_msg)

        except Exception as e:
            self.get_logger().debug(f'Ramp detection error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DualLidarFilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
