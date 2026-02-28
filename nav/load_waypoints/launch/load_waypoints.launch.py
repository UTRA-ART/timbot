from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # --- Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('sim')

    # config_file argument — the waypoints JSON filename
    # Default: sim_waypoints.json
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='sim_waypoints.json',
        description='Name of the waypoints JSON file in load_waypoints/jsons/'
    )

    # only_errors argument — suppresses info-level log output
    only_errors_arg = DeclareLaunchArgument(
        'only_errors',
        default_value='false',
        description='If true, set log level to error instead of info'
    )
    only_errors = LaunchConfiguration('only_errors')
    log_level = PythonExpression([
        "'error' if '", only_errors, "' == 'true' else 'info'"
    ])

    # --- Waypoints File Selection ---
    # Resolved at launch time via PathJoinSubstitution
    waypoints_file = PathJoinSubstitution([
        FindPackageShare('load_waypoints'), 'jsons', LaunchConfiguration('config_file')
    ])

    # --- 1. Waypoint Navigation Server ---
    # Loads JSON waypoints and navigates through them sequentially
    navigate_waypoints_node = Node(
        package='load_waypoints',
        executable='navigate_waypoints.py',
        name='load_waypoints_server',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'waypoints_file': waypoints_file
        }]
    )

    # --- 2. Ramp Navigation ---
    # Detects ramps and handles crossing safely
    ramp_navigate_node = Node(
        package='load_waypoints',
        executable='ramp_navigate.py',
        name='ramp_navigate',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- 3. Navigation Options Service ---
    # Provides /rover_navigation service for manual goal commands
    nav_options_node = Node(
        package='load_waypoints',
        executable='nav_options.py',
        name='nav_control',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        only_errors_arg,
        navigate_waypoints_node,
        ramp_navigate_node,
        nav_options_node,
    ])