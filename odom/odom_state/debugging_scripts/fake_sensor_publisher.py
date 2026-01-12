#!/usr/bin/env python3
"""
Fake Sensor Data Publisher for odom_state Testing

Purpose:
    Publishes simulated sensor data (IMU, wheel odometry, GPS) to test
    the odom_state package EKF nodes without requiring actual hardware
    or Gazebo simulation.

Published Topics:
    - /imu/data (sensor_msgs/Imu): Fake IMU data at 50 Hz
    - /wheel_odom/quat_synced (nav_msgs/Odometry): Fake wheel encoder odometry at 50 Hz
    - /gps/fix (sensor_msgs/NavSatFix): Fake GPS data at 5 Hz

Simulated Motion:
    The robot simulates driving in a circle with configurable parameters.
    
Usage:
    ros2 run odom_state fake_sensor_publisher.py
    
    # With custom parameters:
    ros2 run odom_state fake_sensor_publisher.py --ros-args \
        -p linear_velocity:=0.5 \
        -p angular_velocity:=0.1 \
        -p gps_noise:=0.5

Data Flow Explanation:
    ┌─────────────────────────────────────────────────────────────────────┐
    │                     THIS SCRIPT PUBLISHES                           │
    ├─────────────────────────────────────────────────────────────────────┤
    │  /wheel_odom/quat_synced ─┐                                         │
    │  /imu/data ───────────────┼──► ekf_local ──► /odometry/local        │
    │                           │                        │                │
    │  /gps/fix ──► navsat_transform ──► /odometry/gps   │                │
    │                                          │         │                │
    │                                          └────┬────┘                │
    │                                               ▼                     │
    │                                          ekf_global                 │
    │                                               │                     │
    │                                               ▼                     │
    │                                        /odometry/global             │
    └─────────────────────────────────────────────────────────────────────┘
"""

import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3, TransformStamped
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class FakeSensorPublisher(Node):
    """Publishes fake sensor data for testing odom_state package."""

    def __init__(self):
        super().__init__('fake_sensor_publisher')

        # Declare parameters
        self.declare_parameter('linear_velocity', 0.5)  # m/s
        self.declare_parameter('angular_velocity', 0.1)  # rad/s
        self.declare_parameter('imu_noise', 0.01)  # rad/s noise
        self.declare_parameter('gps_noise', 1.0)  # meters noise
        self.declare_parameter('wheel_noise', 0.02)  # m/s noise
        self.declare_parameter('publish_tf', True)  # Publish odom->base_link TF
        
        # Reference GPS coordinates (Detroit area - matches navsat.yaml)
        self.declare_parameter('ref_latitude', 42.3314)  # degrees
        self.declare_parameter('ref_longitude', -83.0458)  # degrees

        # Get parameters
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.angular_vel = self.get_parameter('angular_velocity').value
        self.imu_noise = self.get_parameter('imu_noise').value
        self.gps_noise = self.get_parameter('gps_noise').value
        self.wheel_noise = self.get_parameter('wheel_noise').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.ref_lat = self.get_parameter('ref_latitude').value
        self.ref_lon = self.get_parameter('ref_longitude').value

        # Robot state (simulated ground truth)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.v_roll = 0.0
        self.v_pitch = 0.0
        self.v_yaw = 0.0

        # Publishers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        self.imu_pub = self.create_publisher(Imu, 'imu/data', qos)
        self.wheel_odom_pub = self.create_publisher(Odometry, 'wheel_odom/quat_synced', qos)
        self.gps_pub = self.create_publisher(NavSatFix, 'gps/fix', qos)
        
        # TF broadcaster (optional)
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Timers
        self.imu_timer = self.create_timer(1.0 / 50.0, self.publish_imu)  # 50 Hz
        self.wheel_timer = self.create_timer(1.0 / 50.0, self.publish_wheel_odom)  # 50 Hz
        self.gps_timer = self.create_timer(1.0 / 5.0, self.publish_gps)  # 5 Hz
        self.state_timer = self.create_timer(1.0 / 100.0, self.update_state)  # 100 Hz state update

        self.last_time = self.get_clock().now()

        self.get_logger().info('='*60)
        self.get_logger().info('Fake Sensor Publisher Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Linear velocity:  {self.linear_vel} m/s')
        self.get_logger().info(f'Angular velocity: {self.angular_vel} rad/s')
        self.get_logger().info(f'Publishing to:')
        self.get_logger().info(f'  - /imu/data (50 Hz)')
        self.get_logger().info(f'  - /wheel_odom/quat_synced (50 Hz)')
        self.get_logger().info(f'  - /gps/fix (5 Hz)')
        if self.publish_tf:
            self.get_logger().info(f'  - TF: odom -> base_link')
        self.get_logger().info('='*60)

    def update_state(self):
        """Update simulated robot state based on velocity commands."""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0 or dt > 1.0:
            return

        # Simulated velocities
        self.vx = self.linear_vel
        self.vy = 0.0
        self.vz = 0.0
        self.v_roll = 0.0
        self.v_pitch = 0.0
        self.v_yaw = self.angular_vel

        # Update pose (simple kinematic model)
        self.yaw += self.v_yaw * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))  # Normalize

        # Velocities in world frame
        world_vx = self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)
        world_vy = self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)

        self.x += world_vx * dt
        self.y += world_vy * dt

    def publish_imu(self):
        """Publish fake IMU data."""
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Orientation (with small noise)
        roll_noisy = self.roll + random.gauss(0, self.imu_noise * 0.1)
        pitch_noisy = self.pitch + random.gauss(0, self.imu_noise * 0.1)
        yaw_noisy = self.yaw + random.gauss(0, self.imu_noise * 0.1)
        msg.orientation = euler_to_quaternion(roll_noisy, pitch_noisy, yaw_noisy)

        # Angular velocity (with noise)
        msg.angular_velocity.x = self.v_roll + random.gauss(0, self.imu_noise)
        msg.angular_velocity.y = self.v_pitch + random.gauss(0, self.imu_noise)
        msg.angular_velocity.z = self.v_yaw + random.gauss(0, self.imu_noise)

        # Linear acceleration (gravity + noise)
        msg.linear_acceleration.x = random.gauss(0, 0.1)
        msg.linear_acceleration.y = random.gauss(0, 0.1)
        msg.linear_acceleration.z = 9.81 + random.gauss(0, 0.1)  # Gravity

        # Covariances (diagonal)
        msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]

        self.imu_pub.publish(msg)

    def publish_wheel_odom(self):
        """Publish fake wheel encoder odometry."""
        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Position (with noise)
        msg.pose.pose.position.x = self.x + random.gauss(0, self.wheel_noise)
        msg.pose.pose.position.y = self.y + random.gauss(0, self.wheel_noise)
        msg.pose.pose.position.z = self.z

        # Orientation
        msg.pose.pose.orientation = euler_to_quaternion(self.roll, self.pitch, self.yaw)

        # Velocity in robot frame (with noise)
        msg.twist.twist.linear.x = self.vx + random.gauss(0, self.wheel_noise)
        msg.twist.twist.linear.y = self.vy + random.gauss(0, self.wheel_noise)
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = self.v_yaw + random.gauss(0, self.wheel_noise * 0.1)

        # Covariance matrices (6x6 flattened)
        pose_cov = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.05, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.05]
        msg.pose.covariance = pose_cov
        msg.twist.covariance = pose_cov

        self.wheel_odom_pub.publish(msg)

        # Optionally publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = self.z
            t.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def publish_gps(self):
        """Publish fake GPS data."""
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'

        # GPS status
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        # Convert x,y to lat/lon (approximate, good enough for testing)
        # 1 degree latitude ≈ 111,139 meters
        # 1 degree longitude ≈ 111,139 * cos(lat) meters
        meters_per_deg_lat = 111139.0
        meters_per_deg_lon = 111139.0 * math.cos(math.radians(self.ref_lat))

        # Add position + noise
        lat_offset = (self.y + random.gauss(0, self.gps_noise)) / meters_per_deg_lat
        lon_offset = (self.x + random.gauss(0, self.gps_noise)) / meters_per_deg_lon

        msg.latitude = self.ref_lat + lat_offset
        msg.longitude = self.ref_lon + lon_offset
        msg.altitude = 200.0 + random.gauss(0, 1.0)  # Altitude with noise

        # Position covariance (meters^2)
        msg.position_covariance = [
            self.gps_noise**2, 0.0, 0.0,
            0.0, self.gps_noise**2, 0.0,
            0.0, 0.0, 4.0  # Higher altitude uncertainty
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.gps_pub.publish(msg)

    def log_state(self):
        """Log current simulated state (for debugging)."""
        self.get_logger().info(
            f'State: x={self.x:.2f}, y={self.y:.2f}, yaw={math.degrees(self.yaw):.1f}°'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
