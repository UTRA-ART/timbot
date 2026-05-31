"""
Simple Nav Isolated Test Launch
================================
Minimal bringup for testing odometry and open-loop waypoint following
without the full Timbot stack. Starts exactly:

  1. Robot State Publisher     — URDF -> /tf (base_link, imu_link, etc.)
  2. Phidgets Spatial          — raw IMU on /imu/data_raw
  3. IMU Relay (NED->ENU)      — corrects orientation frame -> /imu/data
  4. Motor Control             — GPIO PWM drive + Arduino serial encoder read
  5. Wheel Odom Publisher      — integrates encoder ticks -> /wheel_odom
  6. EKF Local                 — fuses /wheel_odom + /imu/data -> /odometry/local
  7. simple_nav_node           — open-loop relative goal follower using /odometry/local

Nothing else (no Nav2, no Cartographer, no GPS, no LiDAR, no cameras).

Usage:
  ros2 launch simple_nav simple_nav.launch.py
  ros2 launch simple_nav simple_nav.launch.py yaw_offset:=5.0
  ros2 launch simple_nav simple_nav.launch.py log_level:=debug
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ComposableNodeContainer
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error',
    )
    log_level = LaunchConfiguration('log_level')

    yaw_offset_arg = DeclareLaunchArgument(
        'yaw_offset',
        default_value='0.0',
        description='IMU heading calibration offset in DEGREES (ENU, about Up).',
    )
    yaw_offset = LaunchConfiguration('yaw_offset')

    orientation_stddev_arg = DeclareLaunchArgument(
        'orientation_stddev',
        default_value='0.05',
        description='IMU orientation covariance diagonal stddev (rad).',
    )
    orientation_stddev = LaunchConfiguration('orientation_stddev')

    # ── Robot description (URDF → /tf static frames) ─────────────────────────

    urdf_path = PathJoinSubstitution([
        FindPackageShare('description'),
        'rover_model',
        'urdf',
        'timbot.urdf.xacro',
    ])

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path, ' sim:=false']),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # ── IMU: Phidgets Spatial + NED->ENU relay ────────────────────────────────

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
                parameters=[{
                    'use_orientation': True,
                    'spatial_algorithm': 'ahrs',
                    'frame_id': 'imu_link',
                    'data_interval_ms': 8,
                    'publish_rate': 0.0,
                    'use_sim_time': False,
                    'ahrs_angular_velocity_threshold': 1.0,
                    'ahrs_angular_velocity_delta_threshold': 0.1,
                    'ahrs_acceleration_threshold': 0.1,
                    'ahrs_mag_time': 10.0,
                    'ahrs_accel_time': 10.0,
                    'ahrs_bias_time': 1.25,
                }],
            ),
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )

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
            'use_sim_time': False,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # ── Wheel encoders: motor control + odom publisher ────────────────────────

    # motor_control reads /cmd_vel and drives GPIO PWM + reads encoder ticks
    # from Arduino serial (/dev/ttyACM0, 115200).
    motor_control = Node(
        package='motor_control',
        executable='motor_control.py',
        name='motor_control_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    # odom_pub integrates /left_wheel/ticks + /right_wheel/ticks
    # into /wheel_odom (nav_msgs/Odometry, odom → base_link).
    wheel_odom = Node(
        package='motor_odom',
        executable='odom_pub.py',
        name='wheel_odom_pub',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    # ── EKF Local: /wheel_odom + /imu/data → /odometry/local ─────────────────

    odom_yaml = PathJoinSubstitution([
        FindPackageShare('odom_state'),
        'config',
        'odom.yaml',
    ])

    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        remappings=[
            ('/odometry/filtered', '/odometry/local'),
            ('set_pose', '/ekf_local/set_pose'),
            ('/set_pose', '/ekf_local/set_pose'),
        ],
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            odom_yaml,
            {'use_sim_time': False},
        ],
    )

    # ── Simple Nav: open-loop relative goal follower ──────────────────────────

    simple_nav_yaml = PathJoinSubstitution([
        FindPackageShare('simple_nav'),
        'config',
        'simple_nav.yaml',
    ])

    simple_nav = Node(
        package='simple_nav',
        executable='simple_nav_node.py',
        name='simple_nav_node',
        output='screen',
        parameters=[simple_nav_yaml],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # ── Launch description ────────────────────────────────────────────────────

    return LaunchDescription([
        log_level_arg,
        yaw_offset_arg,
        orientation_stddev_arg,
        robot_state_publisher,
        phidgets_container,
        imu_relay,
        motor_control,
        wheel_odom,
        ekf_local,
        simple_nav,
    ])
