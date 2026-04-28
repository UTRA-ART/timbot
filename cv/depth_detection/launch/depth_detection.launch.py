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

    params_file = PathJoinSubstitution([
        FindPackageShare('depth_detection'),
        'config',
        LaunchConfiguration('config_file'),
    ])

    depth_node = Node(
        package='depth_detection',
        executable='depth_to_pointcloud.py',
        name='depth_to_pointcloud',
        output='screen',
        parameters=[params_file, {'use_sim_time': LaunchConfiguration('sim')}],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        sim_arg,
        log_level_arg,
        config_file_arg,
        depth_node,
    ])