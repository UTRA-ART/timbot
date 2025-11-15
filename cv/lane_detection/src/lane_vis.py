#!/usr/bin/env python3

import rclpy
from cv.msg import FloatArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
# from pcl_msgs.msg import PointXYZ
from std_msgs.msg import Header

import tf
import tf2_ros
from tf import TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud



class FloatArrayToPointCloud2Node(Node):
    def __init__(self):
        super().__init__("float_array_to_pc2_node")
        self.points2_pub = self.create_publisher(
            PointCloud2, "/cv/lane_cloud", 1
        )

        # listen for transform from camera to lidar frames
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def run(self):
        self.float_array_sub = self.create_subscription(
            FloatArray, "/cv/lane_detections", self.float_array_callback, 10
        )

        rclpy.spin(self)

    def float_array_callback(self, msg):
        points = []
        for float_list in msg.lists:
            for element in float_list.elements:
                point = [element.x, element.y, element.z]
                points.append(point)

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        header = Header(frame_id='bottom_lidar_link', stamp=rclpy.Time.Time())

        output_msg = point_cloud2.create_cloud(header, fields, points)

        trans = self.tf_buffer.lookup_transform('bottom_lidar_link', 'left_camera_link_optical', rclpy.Time.Time(), rclpy.Duration(1))

        output_msg = do_transform_cloud(output_msg, trans)

        self.points2_pub.publish(output_msg)


if __name__ == "__main__":
    float_array_to_pc2_node = FloatArrayToPointCloud2Node()
    float_array_to_pc2_node.run()