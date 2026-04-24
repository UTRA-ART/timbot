#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class DisparityToDepthNode(Node):
    def __init__(self):
        super().__init__('disparity_to_depth')
        self.bridge = CvBridge()
        
        self.sub = self.create_subscription(
            DisparityImage,
            '/disparity',
            self.disparity_callback,
            qos_profile_sensor_data
        )
        self.pub = self.create_publisher(
            Image,
            '/depth/depth_image',
            10
        )
        self.color_sub = self.create_subscription(
            Image,
            '/left/image_raw',
            self.color_callback,
            qos_profile_sensor_data
        )
        self.color_pub = self.create_publisher(
            Image,
            '/left/image_color',
            10
        )
        self.declare_parameter('min_depth', 0.5)
        self.declare_parameter('max_depth', 20.0)
        
        self.get_logger().info("Disparity to Depth node started.")

    def disparity_callback(self, msg: DisparityImage):
        # Extract disparity image (32FC1)
        disparity_cv = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='passthrough')
        
        # Avoid division by zero
        # Depth = (f * T) / disparity
        # Where f is focal length, T is baseline
        valid_mask = disparity_cv > 0.0
        depth_cv = np.zeros_like(disparity_cv, dtype=np.float32)
        
        fT = msg.f * msg.t
        depth_cv[valid_mask] = fT / disparity_cv[valid_mask]
        
        # Apply min and max depth filtering
        min_depth = self.get_parameter('min_depth').value
        max_depth = self.get_parameter('max_depth').value
        
        # Filter out depths outside the requested range
        out_of_bounds = (depth_cv < min_depth) | (depth_cv > max_depth)
        depth_cv[out_of_bounds] = 0.0
        
        # Publish the metric depth image
        depth_msg = self.bridge.cv2_to_imgmsg(depth_cv, encoding='32FC1')
        depth_msg.header = msg.header
        # Use left optical frame since disparity is relative to the left camera
        depth_msg.header.frame_id = "left_camera_link_optical"
        
        self.pub.publish(depth_msg)

    def color_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            color_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            color_msg.header = msg.header
            self.color_pub.publish(color_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert color image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DisparityToDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
