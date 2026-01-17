#!/usr/bin/env python3
"""
Circular Motion Verification Tool for odom_state Testing

Purpose:
    Subscribes to odometry outputs and verifies that the EKF is producing
    the expected circular motion when fed fake sensor data from 
    fake_sensor_publisher.py.

    Expected motion: circular path with:
    - Radius = linear_velocity / angular_velocity
    - For defaults (0.5 m/s, 0.1 rad/s): radius = 5.0 meters

Subscribes to:
    - /odometry/local (nav_msgs/Odometry): Local EKF output
    - /odometry/global (nav_msgs/Odometry): Global EKF output
    - /wheel_odom/quat_synced (nav_msgs/Odometry): Raw wheel odometry (ground truth)

Outputs:
    - Real-time terminal updates with position and circle fit metrics
    - PNG plots saved to /tmp/ when Ctrl+C is pressed
    - Circle fit analysis (center, radius, error)

Usage:
    # Terminal 1: Launch odom_state
    ros2 launch odom_state odom_state.launch.py launch_state:=sim
    
    # Terminal 2: Run fake sensor publisher
    ros2 run odom_state fake_sensor_publisher.py
    
    # Terminal 3: Run this verification script
    ros2 run odom_state verify_circular_motion.py
    
    # Let it run for 30-60 seconds, then Ctrl+C to see results
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from collections import deque
import time


def fit_circle(x_points, y_points):
    """
    Fit a circle to a set of 2D points using least squares.
    Returns: (center_x, center_y, radius, mean_error)
    """
    if len(x_points) < 3:
        return None, None, None, None
    
    x = np.array(x_points)
    y = np.array(y_points)
    
    # Method: Algebraic circle fit
    # Minimize: sum of (x^2 + y^2 - 2*cx*x - 2*cy*y + cx^2 + cy^2 - r^2)^2
    # Rearrange to linear least squares: A @ [cx, cy, c] = b
    # where c = cx^2 + cy^2 - r^2
    
    n = len(x)
    A = np.column_stack([x, y, np.ones(n)])
    b = x**2 + y**2
    
    try:
        # Solve least squares
        result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        cx = result[0] / 2
        cy = result[1] / 2
        r = np.sqrt(result[2] + cx**2 + cy**2)
        
        # Calculate mean error (distance from each point to the fitted circle)
        distances = np.sqrt((x - cx)**2 + (y - cy)**2)
        errors = np.abs(distances - r)
        mean_error = np.mean(errors)
        
        return cx, cy, r, mean_error
    except:
        return None, None, None, None


def quaternion_to_yaw(q):
    """Extract yaw from quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CircularMotionVerifier(Node):
    """Verifies circular motion from odometry data."""

    def __init__(self):
        super().__init__('circular_motion_verifier')
        
        # Parameters
        self.declare_parameter('expected_linear_vel', 0.5)  # m/s
        self.declare_parameter('expected_angular_vel', 0.1)  # rad/s
        self.declare_parameter('max_points', 2000)  # Max points to store
        self.declare_parameter('update_interval', 2.0)  # Seconds between status updates
        
        self.linear_vel = self.get_parameter('expected_linear_vel').value
        self.angular_vel = self.get_parameter('expected_angular_vel').value
        self.max_points = self.get_parameter('max_points').value
        self.update_interval = self.get_parameter('update_interval').value
        
        # Expected radius
        self.expected_radius = abs(self.linear_vel / self.angular_vel) if self.angular_vel != 0 else float('inf')
        
        # Data storage
        self.odom_local_data = {'x': deque(maxlen=self.max_points), 
                                'y': deque(maxlen=self.max_points),
                                'yaw': deque(maxlen=self.max_points),
                                'time': deque(maxlen=self.max_points)}
        self.odom_global_data = {'x': deque(maxlen=self.max_points), 
                                 'y': deque(maxlen=self.max_points),
                                 'yaw': deque(maxlen=self.max_points),
                                 'time': deque(maxlen=self.max_points)}
        self.wheel_odom_data = {'x': deque(maxlen=self.max_points), 
                                'y': deque(maxlen=self.max_points),
                                'yaw': deque(maxlen=self.max_points),
                                'time': deque(maxlen=self.max_points)}
        
        # Subscribers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        self.odom_local_sub = self.create_subscription(
            Odometry, 'odometry/local', self.odom_local_callback, qos)
        self.odom_global_sub = self.create_subscription(
            Odometry, 'odometry/global', self.odom_global_callback, qos)
        self.wheel_odom_sub = self.create_subscription(
            Odometry, 'wheel_odom/quat_synced', self.wheel_odom_callback, qos)
        
        # Status timer
        self.status_timer = self.create_timer(self.update_interval, self.print_status)
        
        self.start_time = time.time()
        
        self.get_logger().info('='*70)
        self.get_logger().info('Circular Motion Verifier Started')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Expected linear velocity:  {self.linear_vel} m/s')
        self.get_logger().info(f'Expected angular velocity: {self.angular_vel} rad/s')
        self.get_logger().info(f'Expected circle radius:    {self.expected_radius:.2f} m')
        self.get_logger().info('='*70)
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  - /odometry/local')
        self.get_logger().info('  - /odometry/global')
        self.get_logger().info('  - /wheel_odom/quat_synced')
        self.get_logger().info('='*70)
        self.get_logger().info('Press Ctrl+C after collecting data to generate plots')
        self.get_logger().info('='*70)

    def store_odom(self, msg, data_dict):
        """Store odometry data."""
        data_dict['x'].append(msg.pose.pose.position.x)
        data_dict['y'].append(msg.pose.pose.position.y)
        data_dict['yaw'].append(quaternion_to_yaw(msg.pose.pose.orientation))
        data_dict['time'].append(time.time() - self.start_time)

    def odom_local_callback(self, msg):
        self.store_odom(msg, self.odom_local_data)

    def odom_global_callback(self, msg):
        self.store_odom(msg, self.odom_global_data)

    def wheel_odom_callback(self, msg):
        self.store_odom(msg, self.wheel_odom_data)

    def analyze_data(self, data_dict, name):
        """Analyze trajectory data and return metrics."""
        if len(data_dict['x']) < 10:
            return None
        
        x = list(data_dict['x'])
        y = list(data_dict['y'])
        
        cx, cy, radius, error = fit_circle(x, y)
        
        if radius is None:
            return None
        
        # Calculate additional metrics
        radius_error = abs(radius - self.expected_radius)
        radius_error_pct = (radius_error / self.expected_radius) * 100 if self.expected_radius > 0 else 0
        
        # Calculate arc length traveled
        total_distance = 0
        for i in range(1, len(x)):
            dx = x[i] - x[i-1]
            dy = y[i] - y[i-1]
            total_distance += math.sqrt(dx*dx + dy*dy)
        
        # Calculate total rotation
        yaw_list = list(data_dict['yaw'])
        total_rotation = 0
        for i in range(1, len(yaw_list)):
            dyaw = yaw_list[i] - yaw_list[i-1]
            # Handle wraparound
            if dyaw > math.pi:
                dyaw -= 2 * math.pi
            elif dyaw < -math.pi:
                dyaw += 2 * math.pi
            total_rotation += dyaw
        
        return {
            'name': name,
            'points': len(x),
            'center_x': cx,
            'center_y': cy,
            'radius': radius,
            'expected_radius': self.expected_radius,
            'radius_error': radius_error,
            'radius_error_pct': radius_error_pct,
            'circle_fit_error': error,
            'total_distance': total_distance,
            'total_rotation_deg': math.degrees(total_rotation),
            'full_circles': abs(total_rotation) / (2 * math.pi)
        }

    def print_status(self):
        """Print current status."""
        elapsed = time.time() - self.start_time
        
        self.get_logger().info('-'*70)
        self.get_logger().info(f'Time elapsed: {elapsed:.1f}s')
        self.get_logger().info(f'Data points: local={len(self.odom_local_data["x"])}, '
                               f'global={len(self.odom_global_data["x"])}, '
                               f'wheel={len(self.wheel_odom_data["x"])}')
        
        # Analyze each source
        for data, name in [(self.wheel_odom_data, 'Wheel Odom'),
                           (self.odom_local_data, 'EKF Local'),
                           (self.odom_global_data, 'EKF Global')]:
            metrics = self.analyze_data(data, name)
            if metrics:
                status = '✓' if metrics['radius_error_pct'] < 20 else '✗'
                self.get_logger().info(
                    f'{status} {name}: R={metrics["radius"]:.2f}m (exp={self.expected_radius:.2f}m, '
                    f'err={metrics["radius_error_pct"]:.1f}%), '
                    f'circles={metrics["full_circles"]:.2f}'
                )

    def generate_plots(self):
        """Generate final analysis plots."""
        try:
            import matplotlib
            matplotlib.use('Agg')  # Non-interactive backend
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().error('matplotlib not installed. Cannot generate plots.')
            self.get_logger().error('Install with: pip install matplotlib')
            return
        
        print('Generating plots...')
        
        # Create figure with subplots
        fig, axes = plt.subplots(2, 2, figsize=(14, 12))
        
        # Plot 1: Trajectory comparison (top-left)
        ax1 = axes[0, 0]
        
        # Draw expected circle - centered at (0, radius) since robot starts at (0,0) facing +X
        # and turns left (positive angular velocity)
        expected_center_x = 0.0
        expected_center_y = self.expected_radius  # Center is perpendicular to initial heading
        theta = np.linspace(0, 2*np.pi, 100)
        expected_x = expected_center_x + self.expected_radius * np.cos(theta)
        expected_y = expected_center_y + self.expected_radius * np.sin(theta)
        ax1.plot(expected_x, expected_y, 'k--', linewidth=2, 
                label=f'Expected (R={self.expected_radius:.2f}m, center=(0,{self.expected_radius:.1f}))', alpha=0.5)
        ax1.scatter(expected_center_x, expected_center_y, color='black', s=100, marker='x', label='Expected center')
        
        # Plot each trajectory
        colors = {'Wheel Odom': 'green', 'EKF Local': 'blue', 'EKF Global': 'red'}
        for data, name in [(self.wheel_odom_data, 'Wheel Odom'),
                           (self.odom_local_data, 'EKF Local'),
                           (self.odom_global_data, 'EKF Global')]:
            if len(data['x']) > 0:
                ax1.plot(list(data['x']), list(data['y']), 
                        color=colors[name], linewidth=1.5, label=name, alpha=0.7)
                # Mark start and end
                ax1.scatter(data['x'][0], data['y'][0], color=colors[name], s=100, marker='o', edgecolors='black')
                ax1.scatter(data['x'][-1], data['y'][-1], color=colors[name], s=100, marker='s', edgecolors='black')
        
        ax1.set_xlabel('X (meters)')
        ax1.set_ylabel('Y (meters)')
        ax1.set_title('Trajectory Comparison')
        ax1.legend()
        ax1.axis('equal')
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Circle fit comparison (top-right)
        ax2 = axes[0, 1]
        
        for data, name in [(self.wheel_odom_data, 'Wheel Odom'),
                           (self.odom_local_data, 'EKF Local'),
                           (self.odom_global_data, 'EKF Global')]:
            metrics = self.analyze_data(data, name)
            if metrics:
                # Draw fitted circle
                theta = np.linspace(0, 2*np.pi, 100)
                fit_x = metrics['center_x'] + metrics['radius'] * np.cos(theta)
                fit_y = metrics['center_y'] + metrics['radius'] * np.sin(theta)
                ax2.plot(fit_x, fit_y, color=colors[name], linewidth=2, 
                        label=f'{name}: R={metrics["radius"]:.2f}m')
                ax2.scatter(metrics['center_x'], metrics['center_y'], 
                           color=colors[name], s=100, marker='+')
        
        # Expected circle - center at (0, radius) for left turn starting at origin
        expected_center_x = 0.0
        expected_center_y = self.expected_radius
        theta = np.linspace(0, 2*np.pi, 100)
        expected_x = expected_center_x + self.expected_radius * np.cos(theta)
        expected_y = expected_center_y + self.expected_radius * np.sin(theta)
        ax2.plot(expected_x, expected_y, 'k--', linewidth=2, 
                label=f'Expected: R={self.expected_radius:.2f}m', alpha=0.5)
        ax2.scatter(expected_center_x, expected_center_y, color='black', s=100, marker='x')  # Expected center
        
        ax2.set_xlabel('X (meters)')
        ax2.set_ylabel('Y (meters)')
        ax2.set_title('Fitted Circles Comparison')
        ax2.legend()
        ax2.axis('equal')
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Position over time (bottom-left)
        ax3 = axes[1, 0]
        
        for data, name in [(self.wheel_odom_data, 'Wheel Odom'),
                           (self.odom_local_data, 'EKF Local'),
                           (self.odom_global_data, 'EKF Global')]:
            if len(data['x']) > 0:
                t = list(data['time'])
                ax3.plot(t, list(data['x']), color=colors[name], linestyle='-', 
                        label=f'{name} X', alpha=0.7)
                ax3.plot(t, list(data['y']), color=colors[name], linestyle='--', 
                        alpha=0.7)
        
        ax3.set_xlabel('Time (seconds)')
        ax3.set_ylabel('Position (meters)')
        ax3.set_title('X (solid) and Y (dashed) Position vs Time')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Plot 4: Error analysis bar chart (bottom-right)
        ax4 = axes[1, 1]
        
        names = []
        radius_errors = []
        fit_errors = []
        
        for data, name in [(self.wheel_odom_data, 'Wheel Odom'),
                           (self.odom_local_data, 'EKF Local'),
                           (self.odom_global_data, 'EKF Global')]:
            metrics = self.analyze_data(data, name)
            if metrics:
                names.append(name)
                radius_errors.append(metrics['radius_error_pct'])
                fit_errors.append(metrics['circle_fit_error'] * 100)  # Convert to cm
        
        if names:
            x_pos = np.arange(len(names))
            width = 0.35
            
            bars1 = ax4.bar(x_pos - width/2, radius_errors, width, label='Radius Error (%)', color='steelblue')
            bars2 = ax4.bar(x_pos + width/2, fit_errors, width, label='Fit Error (cm)', color='coral')
            
            ax4.set_ylabel('Error')
            ax4.set_title('Error Analysis')
            ax4.set_xticks(x_pos)
            ax4.set_xticklabels(names)
            ax4.legend()
            ax4.grid(True, alpha=0.3, axis='y')
            
            # Add value labels on bars
            for bar in bars1:
                height = bar.get_height()
                ax4.annotate(f'{height:.1f}%',
                            xy=(bar.get_x() + bar.get_width() / 2, height),
                            xytext=(0, 3), textcoords="offset points",
                            ha='center', va='bottom', fontsize=9)
            for bar in bars2:
                height = bar.get_height()
                ax4.annotate(f'{height:.1f}cm',
                            xy=(bar.get_x() + bar.get_width() / 2, height),
                            xytext=(0, 3), textcoords="offset points",
                            ha='center', va='bottom', fontsize=9)
        
        plt.tight_layout()
        
        # Save plot
        output_path = '/tmp/odom_circular_motion_analysis.png'
        plt.savefig(output_path, dpi=150)
        print(f'Plot saved to: {output_path}')
        
        # Also save a simple trajectory plot
        fig2, ax = plt.subplots(figsize=(10, 10))
        
        for data, name in [(self.wheel_odom_data, 'Wheel Odom (Ground Truth)'),
                           (self.odom_local_data, 'EKF Local'),
                           (self.odom_global_data, 'EKF Global')]:
            if len(data['x']) > 0:
                ax.plot(list(data['x']), list(data['y']), 
                       color=colors[name.split()[0] + ' ' + name.split()[1] if len(name.split()) > 1 else name], 
                       linewidth=2, label=name, alpha=0.8)
        
        # Expected circle - centered at (0, radius) for left turn from origin
        expected_center_x = 0.0
        expected_center_y = self.expected_radius
        theta = np.linspace(0, 2*np.pi, 100)
        expected_x = expected_center_x + self.expected_radius * np.cos(theta)
        expected_y = expected_center_y + self.expected_radius * np.sin(theta)
        ax.plot(expected_x, expected_y, 'k--', linewidth=2, 
               label=f'Expected Circle (R={self.expected_radius:.2f}m)', alpha=0.5)
        
        ax.set_xlabel('X (meters)', fontsize=12)
        ax.set_ylabel('Y (meters)', fontsize=12)
        ax.set_title('Odometry Trajectory Verification', fontsize=14)
        ax.legend(fontsize=10)
        ax.axis('equal')
        ax.grid(True, alpha=0.3)
        
        trajectory_path = '/tmp/odom_trajectory.png'
        plt.savefig(trajectory_path, dpi=150)
        self.get_logger().info(f'Trajectory plot saved to: {trajectory_path}')
        
        plt.close('all')

    def print_final_report(self):
        """Print final analysis report."""
        print('')
        print('='*70)
        print('FINAL ANALYSIS REPORT')
        print('='*70)
        print(f'Expected radius: {self.expected_radius:.3f} m')
        print(f'(linear_vel={self.linear_vel} m/s / angular_vel={self.angular_vel} rad/s)')
        print('')
        
        for data, name in [(self.wheel_odom_data, 'Wheel Odom (Input)'),
                           (self.odom_local_data, 'EKF Local'),
                           (self.odom_global_data, 'EKF Global')]:
            metrics = self.analyze_data(data, name)
            if metrics:
                print(f'--- {name} ---')
                print(f'  Data points:     {metrics["points"]}')
                print(f'  Fitted radius:   {metrics["radius"]:.3f} m')
                print(f'  Radius error:    {metrics["radius_error"]:.3f} m ({metrics["radius_error_pct"]:.1f}%)')
                print(f'  Circle fit RMSE: {metrics["circle_fit_error"]*100:.2f} cm')
                print(f'  Center:          ({metrics["center_x"]:.3f}, {metrics["center_y"]:.3f})')
                print(f'  Total distance:  {metrics["total_distance"]:.2f} m')
                print(f'  Total rotation:  {metrics["total_rotation_deg"]:.1f}°')
                print(f'  Full circles:    {metrics["full_circles"]:.2f}')
                
                # Pass/Fail verdict
                if metrics["radius_error_pct"] < 10:
                    print(f'  Verdict:         ✓ EXCELLENT (error < 10%)')
                elif metrics["radius_error_pct"] < 20:
                    print(f'  Verdict:         ✓ GOOD (error < 20%)')
                elif metrics["radius_error_pct"] < 50:
                    print(f'  Verdict:         ⚠ MARGINAL (error < 50%)')
                else:
                    print(f'  Verdict:         ✗ POOR (error >= 50%)')
                    print('')
            else:
                print(f'--- {name} ---')
                print(f'  No data received')
                print('')
        
            print('='*70)


def main(args=None):
    rclpy.init(args=args)
    node = CircularMotionVerifier()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('')
        print('Interrupted! Generating final report...')
        node.print_final_report()
        node.generate_plots()
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()


if __name__ == '__main__':
    main()
