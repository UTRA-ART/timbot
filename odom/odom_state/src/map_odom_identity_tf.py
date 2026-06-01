#!/usr/bin/env python3
"""
Publish a static identity transform between map and odom.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros


class MapOdomIdentityTf(Node):
    def __init__(self):
        super().__init__('map_odom_identity_tf')

        self.declare_parameter('enabled', True)
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'odom')

        self.enabled = bool(self.get_parameter('enabled').value)
        if not self.enabled:
            self.get_logger().info('Disabled by parameter; not publishing TF.')
            return

        parent_frame = self.get_parameter('parent_frame').value
        child_frame = self.get_parameter('child_frame').value

        broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        broadcaster.sendTransform(transform)

        self.get_logger().info(
            f'Publishing identity TF: {parent_frame} -> {child_frame}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomIdentityTf()
    if not node.enabled:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return
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
