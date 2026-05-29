from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error',
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='depth_detection.yaml',
        description='Name of the depth detection config file in depth_detection/config/',
    )
    ramp_seg_using_lidar_arg = DeclareLaunchArgument(
        'ramp_seg_using_lidar',
        default_value='false',
        description='If true, /ramp_seg is published by the lidar filter; if false, by this node',
    )

    params_file = PathJoinSubstitution([
        FindPackageShare('depth_detection'),
        'config',
        LaunchConfiguration('config_file'),
    ])

    depth_node = Node(
        package='depth_detection',
        executable='pointcloud_filter_from_rviz.py',
        name='pointcloud_rviz_filter',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': LaunchConfiguration('sim'),
                'ramp_seg_using_lidar': LaunchConfiguration('ramp_seg_using_lidar'),
            },
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        sim_arg,
        log_level_arg,
        config_file_arg,
        ramp_seg_using_lidar_arg,
        depth_node,
    ])
