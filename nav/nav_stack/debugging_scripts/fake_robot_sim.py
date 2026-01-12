#!/usr/bin/env python3
"""
Fake robot simulation for testing nav_stack standalone.
Publishes TF transforms, odometry, and laser scan data.

Usage:
  Terminal 1: ros2 run nav_stack fake_robot_sim.py
  Terminal 2: ros2 launch nav_stack move_base.launch.py use_sim_time:=false
  Terminal 3: ros2 run nav_stack send_nav_goal.py  # or use rviz2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math


class FakeRobotSim(Node):
    def __init__(self):
        super().__init__('fake_robot_sim')
        
        # TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odometry/local', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_modified', 10)
        
        # Subscriber for velocity commands (to simulate robot movement)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/nav_vel', self.cmd_vel_callback, 10)
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vtheta = 0.0
        
        # Publish static transforms
        self.publish_static_transforms()
        
        # Timer for dynamic updates (50 Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Fake Robot Simulation Started!')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Publishing:')
        self.get_logger().info('  - TF: map -> odom -> base_link -> bottom_lidar_link')
        self.get_logger().info('  - /odometry/local (nav_msgs/Odometry)')
        self.get_logger().info('  - /scan_modified (sensor_msgs/LaserScan)')
        self.get_logger().info('Subscribing:')
        self.get_logger().info('  - /nav_vel (geometry_msgs/Twist)')
        self.get_logger().info('=' * 60)

    def publish_static_transforms(self):
        """Publish static transforms that don't change."""
        now = self.get_clock().now().to_msg()
        static_transforms = []
        
        # map -> odom (identity, no drift assumed)
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = now
        t_map_odom.header.frame_id = 'map'
        t_map_odom.child_frame_id = 'odom'
        t_map_odom.transform.rotation.w = 1.0
        static_transforms.append(t_map_odom)
        
        # base_link -> bottom_lidar_link (lidar mounted on robot)
        t_base_lidar = TransformStamped()
        t_base_lidar.header.stamp = now
        t_base_lidar.header.frame_id = 'base_link'
        t_base_lidar.child_frame_id = 'bottom_lidar_link'
        t_base_lidar.transform.translation.x = 0.3  # front of robot
        t_base_lidar.transform.translation.z = 0.2  # height
        t_base_lidar.transform.rotation.w = 1.0
        static_transforms.append(t_base_lidar)
        
        self.static_tf_broadcaster.sendTransform(static_transforms)
        self.get_logger().info('Published static transforms: map->odom, base_link->bottom_lidar_link')

    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands from nav_stack."""
        self.vx = msg.linear.x
        self.vtheta = msg.angular.z
        if abs(self.vx) > 0.01 or abs(self.vtheta) > 0.01:
            self.get_logger().info(f'Received cmd_vel: vx={self.vx:.2f}, vtheta={self.vtheta:.2f}')

    def timer_callback(self):
        dt = 0.02  # 50 Hz
        now = self.get_clock().now().to_msg()
        
        # Update robot pose based on velocity
        self.theta += self.vtheta * dt
        self.x += self.vx * math.cos(self.theta) * dt
        self.y += self.vx * math.sin(self.theta) * dt
        
        # Normalize theta to [-pi, pi]
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        # Publish odom -> base_link transform
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)
        
        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vtheta
        self.odom_pub.publish(odom)
        
        # Publish laser scan (simple: no obstacles, all max range)
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = 'bottom_lidar_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0  # 1 degree resolution
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 30.0
        scan.ranges = [30.0] * 360  # No obstacles
        scan.intensities = [100.0] * 360
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobotSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
