"""
Exports video frames from the ZED camera topic to an MP4 file,
while also displaying the live feed in a window.

Press 'q' in the window to stop recording cleanly.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time


class VideoSaver(Node):
    def __init__(self):
        super().__init__('video_saver')

        self.topic_name = '/zed_node/left/image'

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.listener_callback,
            qos
        )

        self.bridge = CvBridge()
        self.out = None
        self.should_stop = False

        self.get_logger().info(f"Subscribed to {self.topic_name}. Waiting for first frame...")

    def listener_callback(self, msg):
        if self.should_stop:
            return

        # Convert ROS Image to OpenCV BGR
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Initialize VideoWriter on first frame
        if self.out is None:
            height, width = cv_img.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.out = cv2.VideoWriter('sim_capture.mp4', fourcc, 10.0, (width, height))
            self.get_logger().info(f"Recording started at {width}x{height}")

        # Write frame
        self.out.write(cv_img)

        # Display
        cv2.imshow("Recording - ZED Left", cv_img)

        # Check for quit key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Stop requested (q pressed)")
            self.should_stop = True


def main():
    rclpy.init()
    node = VideoSaver()

    try:
        while rclpy.ok() and not node.should_stop:
            rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")

    finally:
        node.get_logger().info("Finalizing video and shutting down...")

        # Give encoder time to flush buffers (important for MP4)
        time.sleep(0.5)

        if node.out is not None:
            node.out.release()

        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()