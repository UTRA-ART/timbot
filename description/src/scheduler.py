#!/usr/bin/env python3
import os
import time
import subprocess
import threading
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix, LaserScan, Imu, Image, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import tf2_ros

class Scheduler(Node):
    '''
    Class to enforce startup order of Timbot ROS 2 nodes 
    '''
    def __init__(self):
        super().__init__('startup_scheduler')
        self.get_logger().info('Scheduler node started.')
        self.subs = []

        # Get parameters
        self.declare_parameter('visual_odom_enable', False)
        self.visual_odom_enable = self.get_parameter('visual_odom_enable').get_parameter_value().bool_value

        # Sensors  
        self.gps_started = False
        self.zed_started = False
        self.imu_started = False
        self.lower_lidar_started = False
        self.upper_lidar_started = False
        self.assign_topic('/gps/fix', 'gps_started', NavSatFix)
        self.assign_topic('/zed_node/left/image_rect_color', 'zed_started', Image)
        self.assign_topic('/imu/data', 'imu_started', Imu)
        self.assign_topic('/scan_lower', 'lower_lidar_started', LaserScan)
        self.assign_topic('/scan_upper','upper_lidar_started', LaserScan)

        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Odometry 
        self.odom_global_published = False
        self.odom_gps_published = False
        self.odom_local_published = False
        self.odom_motor_published = False
        self.assign_topic('/odometry/global', 'odom_global_published', Odometry)
        self.assign_topic('/odometry/gps', 'odom_gps_published', Odometry)
        self.assign_topic('/odometry/local', 'odom_local_published', Odometry)
        self.assign_topic('/fake_odom_wheel_encoder_quat', 'odom_motor_published', Odometry)

        # Cartographer
        self.tracked_pose_set = False
        self.assign_topic('/tracked_pose', 'tracked_pose_set', PoseStamped)

        # Meta
        self.manual_default_set = False
        self.scan_override_set = False 
        self.cv_lane_scan_set = False
        self.merged_scan_set = False
        self.assign_topic('/pause_navigation', 'manual_default_set', Bool, True)
        self.assign_topic('/scan_modified', 'scan_override_set', LaserScan)
        self.assign_topic('/cv/lane_detections_scan','cv_lane_scan_set', LaserScan)
        self.assign_topic('/scan_merged','merged_scan_set', LaserScan)

    def abort(self, msg):
        self.get_logger().error(f"SOMETHING FAILED. ABORTING. THIS CAUSED IT: {msg}")
        # Kill all ROS 2 nodes
        try:
            subprocess.run(['ros2', 'daemon', 'stop'], check=False)
        except Exception as e:
            self.get_logger().error(f"Failed to stop ROS 2 daemon: {e}")

    def assign_topic(self, topic_name: str, bool_name: str, topic_type: Any, expected_value: Optional[Any] = None):
        callback = self.get_topic_callback(bool_name, expected_value)
        subscriber = self.create_subscription(topic_type, topic_name, callback, 10)
        self.subs.append(subscriber)

    def get_topic_callback(self, bool_name: str, expected_val: Optional[Any] = None):
        def callback(msg):
            if expected_val is not None:
                if hasattr(msg, 'data') and msg.data == expected_val:
                    setattr(self, bool_name, True)
            else:
                setattr(self, bool_name, True)
        return callback

    def wait_for_condition(self, bool_name: str, timeout: int = 60):
        start = time.time()
        while not getattr(self, bool_name) and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if time.time() - start >= timeout:
            self.abort(bool_name)
            raise RuntimeError(f'{bool_name} condition not met in time.')

    def wait_for_transform(self, base_frame: str, target_frame: str, timeout: int = 60):
        start = time.time()
        while time.time() - start < timeout:
            try:
                transform = self.tf_buffer.lookup_transform(
                    base_frame, target_frame, rclpy.time.Time()
                )
                self.get_logger().info(f"Transform {base_frame} -> {target_frame} found")
                return True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException):
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
        self.abort(f"Transform {base_frame} -> {target_frame}")
        raise RuntimeError(f'{target_frame} frame not started in time.')

    def launch_command(self, command: str, background: bool = True):
        """Launch ROS 2 command with proper error handling"""
        if background:
            command += " &"
        self.get_logger().info(f"Executing: {command}")
        result = os.system(command)
        if result != 0 and not background:
            self.get_logger().error(f"Command failed with code {result}: {command}")

    def publish_topic_once(self, topic: str, msg_type: str, data: str):
        """Publish a single message to a topic"""
        command = f"ros2 topic pub --once {topic} {msg_type} '{data}'"
        subprocess.Popen(command, shell=True)

    def unsubscribe_all(self):
        for subscriber in self.subs:
            self.destroy_subscription(subscriber)
    
    def run(self):
        """Main scheduler execution"""
        try:
            # Set manual override - publish directly instead of external command
            self.get_logger().info('Setting manual override...')
            
            # Create publisher for manual override
            manual_pub = self.create_publisher(Bool, '/pause_navigation', 10)
            
            # Wait a moment for publisher to be established
            time.sleep(0.5)
            
            # Publish manual override message
            manual_msg = Bool()
            manual_msg.data = True
            manual_pub.publish(manual_msg)
            
            # Allow some time for the message to be processed
            time.sleep(0.1)
            
            # Wait for the subscription callback to trigger
            self.wait_for_condition('manual_default_set', 5)
            self.get_logger().info('Manual mode set to True.')

            # Clean up publisher
            self.destroy_publisher(manual_pub)

            # Launch state publisher
            self.get_logger().info('Initializing state publisher...')
            self.launch_command('ros2 launch description state_publisher.launch.py')
            self.get_logger().info('State publisher succeeded.')

            # Launch sensors
            self.get_logger().info('Starting up sensors...')
            self.launch_command('ros2 launch sensors sensors.launch.py launch_state:=IGVC frame_id:=gps_link')
            self.wait_for_condition('imu_started', 35)
            self.wait_for_condition('lower_lidar_started', 35)
            self.wait_for_condition('upper_lidar_started', 40)
            self.wait_for_condition('gps_started', 90)
            self.get_logger().info('Sensors launched.')

            # Launch scan filter
            self.get_logger().info('Launching scan override...')
            self.launch_command('ros2 launch filter_lidar_data filter_lidar_data.launch.py')
            self.wait_for_condition('scan_override_set', 20)
            self.get_logger().info('Scan overridden.')

            # Launch ZED camera
            self.get_logger().info('Start ZED separately...')
            if self.visual_odom_enable:
                self.launch_command('ros2 launch zed_wrapper zed_no_tf.launch.py position_tracking:=true')
            else:
                self.launch_command('ros2 launch zed_wrapper zed_no_tf.launch.py position_tracking:=false')
            self.wait_for_condition('zed_started', 35)
            self.get_logger().info('ZED launched before CV.')

            # Launch CV pipeline
            self.get_logger().info('Starting CV pipeline...')
            self.launch_command('ros2 launch cv cv_pipeline.launch.py launch_state:=IGVC')
            self.wait_for_condition('cv_lane_scan_set', 70)
            self.wait_for_condition('merged_scan_set', 100)
            self.get_logger().info('CV pipeline launched.')

            # Launch motor odometry
            self.get_logger().info('Starting motor_odom_node...')
            self.launch_command('ros2 launch motor_odom motor_odom.launch.py launch_state:=IGVC')
            self.wait_for_condition('odom_motor_published', 50)
            self.get_logger().info('motor_odom_node launched.')

            # Launch odometry
            self.get_logger().info('Initializing odometry...')
            self.launch_command('ros2 launch odom odom.launch.py launch_state:=IGVC')
            self.wait_for_condition('odom_global_published', 35)
            self.wait_for_condition('odom_local_published', 35)
            self.get_logger().info('Odometry initialized.')

            # Launch UTM transform
            self.get_logger().info('Initializing UTM...')
            self.launch_command('ros2 launch description utm.launch.py')
            self.wait_for_transform('map', 'utm', timeout=120)
            self.wait_for_condition('odom_gps_published', 45)
            self.get_logger().info('UTM initialized.')

            # Launch Cartographer
            self.get_logger().info('Starting cartographer...')
            self.launch_command('ros2 launch description cartographer.launch.py launch_state:=IGVC')
            self.wait_for_condition('tracked_pose_set', 60)
            self.wait_for_transform('odom', 'base_link')
            self.wait_for_transform('map', 'odom')
            self.get_logger().info('Cartographer launched.')

            # Launch navigation stack
            self.get_logger().info('Initializing navigation stack...')
            self.launch_command('ros2 launch nav_stack move_base.launch.py launch_state:=IGVC')
            self.wait_for_transform('base_link', 'map')
            self.get_logger().info('Navigation stack initialized.')

            # Launch RViz
            self.get_logger().info('Starting rviz...')
            self.launch_command('ros2 launch description rviz.launch.py')
            self.get_logger().info('RViz launched.')

            return True

        except Exception as e:
            self.get_logger().error(f"Scheduler failed: {str(e)}")
            return False


def main(args=None):
    rclpy.init(args=args)
    
    try:
        scheduler = Scheduler()
        
        # Run scheduler in a separate thread
        def run_scheduler():
            if scheduler.run():
                scheduler.get_logger().info("Scheduler succeeded. Timbot is ready to rumble!")
                scheduler.unsubscribe_all()
            else:
                scheduler.get_logger().error("Scheduler failed!")
        
        scheduler_thread = threading.Thread(target=run_scheduler)
        scheduler_thread.start()
        
        # Keep the node spinning
        executor = MultiThreadedExecutor()
        executor.add_node(scheduler)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            scheduler.get_logger().info("Scheduler interrupted by user")
        finally:
            scheduler_thread.join(timeout=5)
            
    except Exception as e:
        print(f"Failed to initialize scheduler: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()