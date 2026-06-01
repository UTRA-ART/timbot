#!/usr/bin/env python3

import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, TransformException


class PointCloudFrameRelay(Node):
    def __init__(self):
        super().__init__('pointcloud_frame_relay')

        self.declare_parameter('input_topic', '/zed_node/left/points')
        self.declare_parameter('output_topic', '/zed_node/left/points_rviz')
        self.declare_parameter('output_frame_id', 'base_link')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Relaying {self.input_topic} -> {self.output_topic} '
            f'(transforming to frame: {self.output_frame_id})'
        )

    def pointcloud_callback(self, msg: PointCloud2):
        src_frame = msg.header.frame_id

        if src_frame == self.output_frame_id:
            self.publisher.publish(msg)
            return

        # Try stamped lookup first, fall back to latest if TF isn't caught up yet
        tf = None
        for stamp in (msg.header.stamp, rclpy.time.Time()):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.output_frame_id,
                    src_frame,
                    stamp,
                    timeout=rclpy.duration.Duration(seconds=0.05),
                )
                break
            except TransformException:
                continue

        if tf is None:
            self.get_logger().warn(
                f'TF lookup {src_frame} -> {self.output_frame_id} failed, dropping cloud',
                throttle_duration_sec=2.0,
            )
            return

        r = tf.transform.rotation
        qx, qy, qz, qw = r.x, r.y, r.z, r.w
        R = np.array([
            [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [    2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
            [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
        ], dtype=np.float64)
        t = tf.transform.translation
        trans = np.array([t.x, t.y, t.z], dtype=np.float64)

        raw = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not raw:
            return

        pts = np.array([(p[0], p[1], p[2]) for p in raw], dtype=np.float64)  # (N, 3)
        transformed = (R @ pts.T).T + trans      # (N, 3)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.output_frame_id
        self.publisher.publish(point_cloud2.create_cloud_xyz32(header, transformed.tolist()))


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
