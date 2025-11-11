#!/usr/bin/env python3
import struct
import sys 
import time
import json
import os

import cv2
import numpy as np
import onnx
import onnxruntime as ort 
import torch
import pandas as pd

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# import rospkg
# import rospy

from std_msgs.msg import Header
from cv.msg import FloatArray, FloatList
from geometry_msgs.msg import Point
from cv_utils import camera_projection

from line_fitting import fit_lanes
import ultralytics
from ultralytics import YOLO

from threshold_lane.threshold import lane_detection

# import open3d as o3d
from sensor_msgs.msg import CameraInfo, LaserScan, PointCloud2, PointField
# from sensor_msgs import point_cloud2
from torch.quantization import quantize_dynamic

# ROS 2 specific imports
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from sensor_msgs_py import point_cloud2


class CVModelInferencer(Node):
    def __init__(self):
        # rospy.init_node('lane_detection_model_inference')
        super().__init__('lane_detection_model_inference')
        
        # self.pub = rospy.Publisher('cv/lane_detections', FloatArray, queue_size=10)
        # self.pub_raw = rospy.Publisher('cv/model_output', Image, queue_size=10)
        # self.pub_pt = rospy.Publisher('cv/lane_detections_cloud', PointCloud2, queue_size=10)
        # self.pub_scan = rospy.Publisher('cv/lane_detections_scan', LaserScan, queue_size=10)
        self.pub = self.create_publisher(FloatArray, 'cv/lane_detections', 10)
        self.pub_raw = self.create_publisher(Image, 'cv/model_output', 10)
        self.pub_pt = self.create_publisher(PointCloud2, 'cv/lane_detections_cloud', 10)
        # self.pub_scan = self.create_publisher(LaserScan, 'cv/lane_detections_scan', 10)

        self.bridge = CvBridge()
        self.projection = camera_projection.CameraProjection()
        
        # rospack = rospkg.RosPack()
        # self.model_path = rospack.get_path('lane_detection') + '/models/best_model_int8.pt'
        # self.depth_map_path = rospack.get_path('lane_detection') + '/config/depth_map.npy'
        share_dir = get_package_share_directory('lane_detection')
        self.model_path = os.path.join(share_dir, 'models', 'best_model_int8.pt')
        self.depth_map_path = os.path.join(share_dir, 'config', 'depth_map.npy')

        # Get the parameter to decide between deep learning and classical
        # self.classical_mode = rospy.get_param('/lane_detection_inference/lane_detection_mode')
        self.declare_parameter('lane_detection_mode', 0)
        self.classical_mode = int(self.get_parameter('lane_detection_mode').value)

        self.Inference = None
        self.lane_detection = None
    
        if self.classical_mode == 1:
            self.lane_detection = lane_detection
            # rospy.loginfo("Lane Detection node initialized with CLASSICAL... ")
            self.get_logger().info("Lane Detection node initialized with CLASSICAL... ")
        else:
            self.Inference = YOLO(self.model_path)
            # self.Inference = quantize_dynamic(self.Inference, {torch.nn.Linear}, dtype=torch.qint8)
            # self.Inference = Inference(self.model_path, False)

            # rospy.loginfo("Lane Detection node initialized with DEEP LEARNING...\nCUDA status: %s ", torch.cuda.is_available())
            self.get_logger().info(f"Lane Detection node initialized with DEEP LEARNING...\nCUDA status: {torch.cuda.is_available()}")

        # listen for transform from camera to lidar frames
        # self.listener = tf.TransformListener()
        # self.listener.waitForTransform("/left_camera_link_optical", "/base_laser", rospy.Time(), rospy.Duration(10.0))
        
        # Frame skipping logic to reduce computation load (30 fps vs 5 fps, every 5th frame is processes)
        self.frame_count = 0
        self.frame_skip = 5

        # # Sets node rate to 5 Hz
        # self.rate = rospy.Rate(9)
        
    def run(self):
        # Ensures only latest frame is processed, mitigates lag
        # rospy.Subscriber("/image", Image, self.process_image, queue_size=1)
        self.create_subscription(Image, "/image", self.process_image, qos_profile_sensor_data)
        # rospy.spin()
        rclpy.spin(self)
   
    def lane_transform(self, img):
        length = img.shape[0]
        width = img.shape[1]
        new_width = int(width/8)


        input_pts = np.float32([[int(width/2-new_width),0], 
                                [int(width/2+new_width),0], 
                                [width,length],
                                [0,length] ])
        output_pts = np.float32([[new_width, 0],
                                [width-new_width, 0],
                                [int(width/2)+new_width,length],
                                [int(width/2)-new_width,length]])
        M2 = cv2.getPerspectiveTransform(input_pts,output_pts)
        out = cv2.warpPerspective(img,M2,(width, length),flags=cv2.INTER_LINEAR)
        return out


    def process_image(self, data):
        if data == []:
            return
        # Frame skipping logic
        # self.frame_count += 1
        # if self.frame_count % self.frame_skip != 0:
        #     return
        # self.frame_count = 0
            
        raw = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        projected_lanes = np.load(self.depth_map_path)

        
        if raw is not None:
            # Get the image
            input_img = raw.copy()
            input_img = cv2.resize(input_img, (330, 180))
            input_img = input_img[:,:,:3]
            
            # Do model inference 
            output = None
            mask = None

            if self.classical_mode:
                output = self.lane_detection(input_img)

                mask = np.where(output > 0.5, 1., 0.)
                mask = mask.astype(np.uint8)

            else:
                # output = self.Inference.inference(input_img)
                # cv2.rectangle(input_img, (0,0), (input_img.shape[1],int(input_img.shape[0] / 9)), (0,0,0), -1) 
            
                output = self.Inference(input_img)
                confidence_threshold = 0.5

                output_image = np.zeros_like(input_img[:,:,0], dtype=np.uint8)

                if output and output[0].masks:
                    for k in range(len(output[0].masks)):
                        mask = np.array(output[0].masks[k].data.cpu() if torch.cuda.is_available() else output[0].masks[k].data)  # Convert tensor to numpy array
                        label = output[0].names[int(output[0].boxes[k].cls)]

                        if float(output[0].boxes[k].conf) > confidence_threshold:  # Check confidence level
                            if label == 'lane':
                                img = np.where(mask > 0.5, 255, 0).astype(np.uint8)
                                img = cv2.resize(img.squeeze(), (output_image.shape[1], output_image.shape[0]))
                                output_image = np.maximum(output_image, img)

                output = output_image

            # Publish to /cv/model_output
            img_msg = self.bridge.cv2_to_imgmsg(output, encoding='passthrough')
            img_msg.header.stamp = data.header.stamp
            # img_msg.header.stamp = data.header.stamp
            if img_msg is not None:
                self.pub_raw.publish(img_msg)
            
            # Build the message
            lane_msg = FloatList()
            pts_msg = []
            cloud = []

            for i in range(output.shape[0]):
                for j in range(output.shape[1]):
                    if((output[i][j])==255):
                        pt_msg = Point()
                        pt_msg.x = float(projected_lanes[i][j][0])
                        pt_msg.y = float(projected_lanes[i][j][1])
                        pt_msg.z = float(projected_lanes[i][j][2])

                        pts_msg.append(pt_msg)
                        cloud.append((pt_msg.x, pt_msg.y, pt_msg.z))

            lane_msg.elements = pts_msg


            msg_header = Header(frame_id='left_camera_link_optical')
            msg = FloatArray(header=msg_header, lists=[lane_msg])
            msg.header.stamp = data.header.stamp
            self.pub.publish(msg)

            pt_header = Header(frame_id='left_camera_link_optical')
            pt_header.stamp = data.header.stamp
            # pt_cloud = point_cloud2.create_cloud_xyz32(header=pt_header, points=cloud)
            pt_cloud = point_cloud2.create_cloud_xyz32(pt_header, cloud)
            self.pub_pt.publish(pt_cloud)

            # Contols publishing rate
            # self.rate.sleep()
                


def main(args=None):
    rclpy.init(args=args)
    wrapper = CVModelInferencer()
    try:
        wrapper.run()
    except KeyboardInterrupt:
        pass
    finally:
        wrapper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
