"""
IMU Test Launch — Phidgets AHRS or BNO085 (selectable)
=======================================================
Reads defaults from launch/config/imu_test.yaml. Edit that file to tune,
then copy final values to comp.yaml when satisfied.

Phidgets mode (use_new_imu: false):
  1. phidgets_spatial (onboard AHRS, orientation in NED) on /imu/data_raw.
  2. imu_relay (odom_state): NED->ENU correction -> /imu/data + imu_ned/imu_enu.

BNO085 mode (use_new_imu: true):
  1. bno085_imu (odom_state): already ENU, publishes /imu/data + imu_enu directly.

Usage:
  ros2 launch timbot_launch imu_test.launch.py
  ros2 launch timbot_launch imu_test.launch.py use_new_imu:=true
  ros2 launch timbot_launch imu_test.launch.py use_new_imu:=true port:=/dev/ttyACM1

Calibration:
  ros2 topic echo /imu_enu    # z = yaw in degrees (East=0, North=90)
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _load_config(config_file: str) -> dict:
    config_path = os.path.join(
        get_package_share_directory('timbot_launch'),
        'config',
        config_file,
    )
    with open(config_path) as f:
        return yaml.safe_load(f) or {}


def _launch_nodes(context, *args, **kwargs):
    config_file = LaunchConfiguration('config').perform(context)
    cfg = _load_config(config_file)

    # Command-line args override the YAML
    def _get(arg_name, yaml_key, default, cast=str):
        val = LaunchConfiguration(arg_name).perform(context)
        if val != '__from_yaml__':
            return cast(val)
        raw = cfg
        for k in yaml_key.split('.'):
            raw = raw.get(k, None) if isinstance(raw, dict) else None
        return cast(raw) if raw is not None else cast(default)

    use_new_imu   = _get('use_new_imu',   'use_new_imu',            False,  lambda v: str(v).lower() == 'true')
    port          = _get('port',           'new_imu_port',           '/dev/ttyACM1')
    baud          = _get('baud',           'new_imu_baud',           115200, int)
    new_yaw_off   = _get('new_yaw_offset', 'new_imu_yaw_offset',     0.0,    float)
    old_yaw_off   = _get('yaw_offset',     'imu_relay.yaw_offset',   0.0,    float)
    orient_stddev = _get('orientation_stddev', 'imu_relay.orientation_stddev', 0.05, float)

    use_sim_time  = LaunchConfiguration('sim')
    log_level     = LaunchConfiguration('log_level').perform(context)

    if use_new_imu:
        return [
            Node(
                package='odom_state',
                executable='bno085_imu.py',
                name='bno085_imu',
                output='screen',
                parameters=[{
                    'port': port,
                    'baud': baud,
                    'yaw_offset': new_yaw_off,
                    'use_sim_time': use_sim_time,
                }],
                arguments=['--ros-args', '--log-level', log_level],
            ),
        ]

    phidgets_params = {
        'use_orientation': True,
        'spatial_algorithm': 'ahrs',
        'frame_id': 'imu_link',
        'data_interval_ms': 8,
        'publish_rate': 0.0,
        'use_sim_time': use_sim_time,
        'ahrs_angular_velocity_threshold': 1.0,
        'ahrs_angular_velocity_delta_threshold': 0.1,
        'ahrs_acceleration_threshold': 0.1,
        'ahrs_mag_time': 10.0,
        'ahrs_accel_time': 10.0,
        'ahrs_bias_time': 1.25,
    }

    return [
        ComposableNodeContainer(
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
        ),
        Node(
            package='odom_state',
            executable='imu_relay.py',
            name='imu_relay',
            output='screen',
            parameters=[{
                'input_topic': '/imu/data_raw',
                'output_topic': '/imu/data',
                'yaw_offset': old_yaw_off,
                'orientation_stddev': orient_stddev,
                'use_sim_time': use_sim_time,
            }],
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='imu_test.yaml',
                              description='Config file in timbot_launch/config/'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='Use simulation clock'),
        DeclareLaunchArgument('log_level', default_value='info',
                              description='Log level: debug, info, warn, error'),
        # All overrides default to sentinel so we know to read from YAML instead
        DeclareLaunchArgument('use_new_imu',         default_value='__from_yaml__'),
        DeclareLaunchArgument('port',                default_value='__from_yaml__'),
        DeclareLaunchArgument('baud',                default_value='__from_yaml__'),
        DeclareLaunchArgument('new_yaw_offset',      default_value='__from_yaml__',
                              description='Yaw offset in degrees for BNO085'),
        DeclareLaunchArgument('yaw_offset',          default_value='__from_yaml__',
                              description='Yaw offset in degrees for phidgets relay'),
        DeclareLaunchArgument('orientation_stddev',  default_value='__from_yaml__',
                              description='Orientation stddev in rad (phidgets only)'),
        OpaqueFunction(function=_launch_nodes),
    ])
