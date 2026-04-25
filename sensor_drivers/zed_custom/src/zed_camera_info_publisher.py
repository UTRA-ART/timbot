#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import qos_profile_sensor_data

class ZedCameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('zed_camera_info_publisher')

        self.declare_parameter('left_frame_id', 'left_camera_link_optical')
        self.declare_parameter('right_frame_id', 'left_camera_link_optical')
        self.declare_parameter('fx', 350.0)
        self.declare_parameter('fy', 350.0)
        self.declare_parameter('cx', 336.0)
        self.declare_parameter('cy', 188.0)
        self.declare_parameter('baseline', 0.12)

        self.left_sub = self.create_subscription(Image, '/left/image_raw', self.left_cb, qos_profile_sensor_data)
        self.right_sub = self.create_subscription(Image, '/right/image_raw', self.right_cb, qos_profile_sensor_data)
        
        self.left_info_pub = self.create_publisher(CameraInfo, '/left/camera_info', qos_profile_sensor_data)
        self.right_info_pub = self.create_publisher(CameraInfo, '/right/camera_info', qos_profile_sensor_data)
        
        # Approximate ZED WVGA intrinsics (672x376), configurable via ROS params.
        fx = float(self.get_parameter('fx').value)
        fy = float(self.get_parameter('fy').value)
        cx = float(self.get_parameter('cx').value)
        cy = float(self.get_parameter('cy').value)
        baseline = float(self.get_parameter('baseline').value)
        self.left_frame_id = str(self.get_parameter('left_frame_id').value)
        self.right_frame_id = str(self.get_parameter('right_frame_id').value)
        
        # Left CameraInfo
        self.left_info = CameraInfo()
        self.left_info.header.frame_id = self.left_frame_id
        self.left_info.width = 672
        self.left_info.height = 376
        self.left_info.distortion_model = 'plumb_bob'
        self.left_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.left_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.left_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.left_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        # Right CameraInfo
        self.right_info = CameraInfo()
        # Stereo can operate with this setup while right baseline is encoded in P[3].
        self.right_info.header.frame_id = self.right_frame_id
        self.right_info.width = 672
        self.right_info.height = 376
        self.right_info.distortion_model = 'plumb_bob'
        self.right_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.right_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.right_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # Tx = -fx * baseline
        tx = -fx * baseline
        self.right_info.p = [fx, 0.0, cx, tx, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    def left_cb(self, msg):
        self.left_info.header.stamp = msg.header.stamp
        self.left_info.header.frame_id = self.left_frame_id
        self.left_info_pub.publish(self.left_info)

    def right_cb(self, msg):
        self.right_info.header.stamp = msg.header.stamp
        self.right_info.header.frame_id = self.right_frame_id
        self.right_info_pub.publish(self.right_info)

def main(args=None):
    rclpy.init(args=args)
    node = ZedCameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
