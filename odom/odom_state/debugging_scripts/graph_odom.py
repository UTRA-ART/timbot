#!/usr/bin/env python3
"""
Odometry Visualization and Error Analysis Tool

Purpose:
    Compares estimated robot position (from tracking/localization) against 
    ground truth data from Gazebo simulation. Generates RMSE plots to 
    quantify localization accuracy.

Subscribes to:
    - ground_truth_odom (nav_msgs/Odometry): Perfect position from simulator
    - tracked_pose (geometry_msgs/PoseStamped): Estimated position from localization

Outputs:
    - PNG plots saved to /tmp/ showing position over time and RMSE errors
    
Usage:
    ros2 run odom_state graph_odom
    (Run during simulation, then Ctrl+C to generate plots)
"""

import math

import matplotlib.pyplot as plt
import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class OdomPlotterNode(rclpy.Node):
    def __init__(self):
        super().__init__('odom_plotter')

        # Create subscribers for ground truth and estimated position
        self.ground_truth_sub = message_filters.Subscriber(
            self, Odometry, "ground_truth_odom", 10
        )
        self.tracked_pose_sub = message_filters.Subscriber(
            self, PoseStamped, "tracked_pose", 10
        )

        # Synchronize messages by timestamp to ensure we're comparing same moments
        self.time_sync = message_filters.TimeSynchronizer(
            [self.ground_truth_sub, self.tracked_pose_sub], 10
        )

        self.time_sync.registerCallback(self.on_callback)

        # Store all received data for plotting after node shuts down
        self.ground_truth_states = []
        self.tracked_poses = []

    def on_callback(self, odom_msg, pose_msg):
        """Collect synchronized ground truth and estimated pose data"""
        self.ground_truth_states.append(odom_msg)
        self.tracked_poses.append(pose_msg)
    
    def plot(self):
        """Generate comparison plots and RMSE analysis after data collection"""
        # Extract ground truth positions
        gt_x, gt_y, gt_z = [], [], []
        for gt_odom in self.ground_truth_states:
            gt_x.append(gt_odom.pose.position.x)
            gt_y.append(gt_odom.pose.position.y)
            gt_z.append(gt_odom.pose.position.z)

        # Extract estimated positions
        tracked_x, tracked_y, tracked_z = [], [], []
        for tracked in self.tracked_poses:
            tracked_x.append(tracked.pose.position.x)
            tracked_y.append(tracked.pose.position.y)
            tracked_z.append(tracked.pose.position.z)

        # Plot X position over time (ground truth vs estimated)
        N = range(len(self.tracked_poses))
        plt.plot(N, tracked_x, label="cartographer")
        plt.plot(N, gt_x, label="gt")
        plt.xlabel("Timestep")
        plt.ylabel("x position")
        plt.title("x vs timestep")
        plt.legend()
        plt.savefig("/tmp/x_position.png")
        plt.clf()

        plt.plot(N, tracked_y, label="cartographer")
        plt.plot(N, gt_y, label="gt")
        plt.xlabel("Timestep")
        plt.ylabel("y position")
        plt.title("y vs timestep")
        plt.legend()
        plt.savefig("/tmp/y_position.png")
        plt.clf()

        # Calculate and plot X position error (RMSE over time)
        x_rmse = list(math.sqrt((x - x_hat) ** 2) for x, x_hat in zip(gt_x, tracked_x))
        plt.plot(N, x_rmse)
        plt.xlabel("Timestep")
        plt.ylabel("rmse")
        plt.title("x rmse vs timestep")
        plt.savefig("/tmp/x_rmse.png")
        plt.clf()

        # Calculate and plot Y position error (RMSE over time)
        y_rmse = list(math.sqrt((y - y_hat) ** 2) for y, y_hat in zip(gt_y, tracked_y))
        plt.plot(N, y_rmse)
        plt.xlabel("Timestep")
        plt.ylabel("rmse")
        plt.title("y rmse vs timestep")
        plt.savefig("/tmp/y_rmse.png")
        plt.clf()

        # Uncomment to save plots in files
        """
        plt.title("Cartographer position (2d)")
        plt.plot(tracked_x, tracked_y)
        plt.xlabel("x")
        plt.ylabel("y")
        plt.savefig("/tmp/cartographer_position.png")
        plt.clf()

        plt.title("Ground truth position (2d)")
        plt.plot(gt_x, gt_y)
        plt.xlabel("x")
        plt.ylabel("y")
        plt.savefig("/tmp/ground_truth_position.png")
        plt.clf()
        """

def main(args = None):
    rclpy.init(args=args)

    odom_plotter_node = OdomPlotterNode()
    rclpy.spin(odom_plotter_node)

    odom_plotter_node.plot()
    
    odom_plotter_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()