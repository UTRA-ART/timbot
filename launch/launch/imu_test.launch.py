"""
IMU Test Launch — Phidgets AHRS + NED->ENU relay ONLY
=====================================================
Standalone bringup for bench-testing / calibrating the IMU without launching
the full Timbot stack. Starts:

  1. phidgets_spatial with the onboard AHRS enabled (orientation in NED) on
     /imu/data_raw.
  2. imu_relay (odom_state): NED->ENU correction -> /imu/data, plus the degree
     debug topics imu_ned / imu_enu.

Usage:
  ros2 launch timbot_launch imu_test.launch.py
  ros2 launch timbot_launch imu_test.launch.py yaw_offset:=12.5

Calibration:
  ros2 topic echo /imu_enu           # absolute orientation (deg) on /imu/data
  ros2 param set /imu_relay yaw_offset <deg>   # live heading tweak (no relaunch)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Use simulation clock if true (real rover: false)',
    )
    use_sim_time = LaunchConfiguration('sim')

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error',
    )
    log_level = LaunchConfiguration('log_level')

    yaw_offset_arg = DeclareLaunchArgument(
        'yaw_offset',
        default_value='0.0',
        description='World-frame yaw calibration offset in DEGREES (about ENU Up).',
    )
    yaw_offset = LaunchConfiguration('yaw_offset')

    orientation_stddev_arg = DeclareLaunchArgument(
        'orientation_stddev',
        default_value='0.05',
        description='Orientation covariance (diagonal stddev, rad).',
    )
    orientation_stddev = LaunchConfiguration('orientation_stddev')

    # --- Phidgets Spatial with onboard AHRS (orientation in NED) ---
    phidgets_params = {
        'use_orientation': True,        # enable on-device AHRS fusion
        'spatial_algorithm': 'ahrs',
        'frame_id': 'imu_link',
        # 125 Hz is the magnetometer ceiling; matches the URDF imu_update_rate.
        'data_interval_ms': 8,
        'publish_rate': 0.0,            # publish on every device sample
        'use_sim_time': use_sim_time,

        # AHRS tuning (Phidgets stock defaults).
        'ahrs_angular_velocity_threshold': 1.0,
        'ahrs_angular_velocity_delta_threshold': 0.1,
        'ahrs_acceleration_threshold': 0.1,
        'ahrs_mag_time': 10.0,
        'ahrs_accel_time': 10.0,
        'ahrs_bias_time': 1.25,
    }

    phidgets_container = ComposableNodeContainer(
        name='phidget_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='phidgets_spatial',
                plugin='phidgets::SpatialRosI',
                name='phidgets_spatial',
                parameters=[phidgets_params],
            ),
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )

    # --- Relay: NED orientation -> ENU on /imu/data + imu_ned / imu_enu (deg) ---
    imu_relay = Node(
        package='odom_state',
        executable='imu_relay.py',
        name='imu_relay',
        output='screen',
        parameters=[{
            'input_topic': '/imu/data_raw',
            'output_topic': '/imu/data',
            'yaw_offset': yaw_offset,
            'orientation_stddev': orientation_stddev,
            'use_sim_time': use_sim_time,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        sim_arg,
        log_level_arg,
        yaw_offset_arg,
        orientation_stddev_arg,
        phidgets_container,
        imu_relay,
    ])
