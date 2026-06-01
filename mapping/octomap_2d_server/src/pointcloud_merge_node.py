#!/usr/bin/env python3

"""Republish LaserScan and depth PointCloud2 on a shared topic.

- Converts the LaserScan to a point cloud in its native frame.
- Republishes the depth cloud as-is.
- Lets OctoMap handle TF to compute the sensor origin per message.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

from laser_geometry import LaserProjection


class PointCloudMergeNode(Node):
    def __init__(self):
        super().__init__('pointcloud_merge_node')

        self.declare_parameter('scan_topic', '/laser_scan')
        self.declare_parameter('depth_cloud_topic', '/zed_node/left/obstacle_points')
        self.declare_parameter('output_topic', '/combined_cloud')
        self.declare_parameter('output_frame', 'odom')

        self.scan_topic = self.get_parameter('scan_topic').value
        self.depth_cloud_topic = self.get_parameter('depth_cloud_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('output_frame').value

        self.laser_projector = LaserProjection()
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.depth_sub = self.create_subscription(
            PointCloud2,
            self.depth_cloud_topic,
            self.depth_callback,
            qos_profile_sensor_data,
        )

        self.publisher = self.create_publisher(PointCloud2, self.output_topic, qos_profile_sensor_data)

        self.get_logger().info(
            f'Republishing {self.scan_topic} and {self.depth_cloud_topic} -> {self.output_topic}'
        )

    def scan_callback(self, scan_msg: LaserScan):
        scan_cloud = self.laser_projector.projectLaser(scan_msg)
        if scan_cloud.width == 0:
            return
        self.publisher.publish(scan_cloud)

    def depth_callback(self, depth_msg: PointCloud2):
        if depth_msg.width == 0:
            return
        self.publisher.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMergeNode()
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
