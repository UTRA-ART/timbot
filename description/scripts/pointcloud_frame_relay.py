#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class PointCloudFrameRelay(Node):
    def __init__(self):
        super().__init__('pointcloud_frame_relay')

        self.declare_parameter('input_topic', '/zed_node/left/points')
        self.declare_parameter('output_topic', '/zed_node/left/points_rviz')
        self.declare_parameter('output_frame_id', 'left_camera_link')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value

        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            10,
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Relaying point cloud {self.input_topic} -> {self.output_topic} '
            f'with frame_id={self.output_frame_id}'
        )

    def pointcloud_callback(self, msg: PointCloud2):
        relay_msg = copy.deepcopy(msg)
        relay_msg.header.frame_id = self.output_frame_id
        self.publisher.publish(relay_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFrameRelay()
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
