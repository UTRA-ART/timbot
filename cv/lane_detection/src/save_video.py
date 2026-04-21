"""
Exports video frames from the ZED camera topic to an MP4 file, while also displaying the live feed in a window.
Press 'q' to stop recording and close the window.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoSaver(Node):
    def __init__(self):
        super().__init__('video_saver')
        
        # Updated to match your screenshot
        self.topic_name = '/zed_node/left/image'
        
        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.listener_callback,
            10)
        
        self.bridge = CvBridge()
        self.out = None
        
        self.get_logger().info(f"Subscribed to {self.topic_name}. Waiting for first frame...")

    def listener_callback(self, msg):
        # Convert ROS Image to OpenCV BGR
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Initialize the VideoWriter only after we get the first frame
        # This automatically handles the resolution (Width/Height)
        if self.out is None:
            height, width = cv_img.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.out = cv2.VideoWriter('sim_capture.mp4', fourcc, 20.0, (width, height))
            self.get_logger().info(f"Recording started at {width}x{height}")

        self.out.write(cv_img)
        
        # Display window (press 'q' here or Ctrl+C in terminal to stop)
        cv2.imshow("Recording - ZED Left", cv_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def __del__(self):
        if self.out is not None:
            self.out.release()
            cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = VideoSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Explicit release to ensure the file is saved properly
        if node.out:
            node.out.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()