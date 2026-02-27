from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # --- Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('sim')

    # mute_warnings argument — suppresses warning-level log output
    mute_warnings_arg = DeclareLaunchArgument(
        'mute_warnings',
        default_value='false',
        description='If true, set log level to error instead of warn'
    )
    mute_warnings = LaunchConfiguration('mute_warnings')
    log_level = PythonExpression([
        "'error' if '", mute_warnings, "' == 'true' else 'warn'"
    ])

    # --- Parameters based on sim/real ---
    # In sim, use 20 degrees for ramp detection; in real, use 45 degrees
    max_theta = PythonExpression([
        "20.0 if '", use_sim_time, "' == 'true' else 45.0"
    ])

    # Upper lidar stop index: 1080 for sim, 1145 for real
    upper_stop_index = PythonExpression([
        "1080 if '", use_sim_time, "' == 'true' else 1145"
    ])

    # --- Dual LiDAR Filter Node ---
    # Compares upper and lower LiDAR to detect ramps
    # Publishes /scan_modified (filtered) and /ramp_seg (ramp points)
    dual_lidar_filter_node = Node(
        package='filter_lidar_data',
        executable='dual_lidar_filter_node.py',
        name='dual_lidar_filter',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'main_lidar_topic': '/scan_lower',
            'upper_lidar_topic': '/scan_upper', 
            'out_topic': '/scan_modified',
            'distance_to_second_lidar': 0.14,
            'max_theta_degrees': max_theta,
            'compare_lidar_time_tolerance_seconds': 2,
            'upper_lidar_start_index': 0,
            'upper_lidar_stop_index': upper_stop_index,
            'upper_lidar_angular_total_range': 360,
            'main_lidar_angular_total_range': 360,
            'limit_output_range': True,
            'desired_output_total_range': 180,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        mute_warnings_arg,
        dual_lidar_filter_node,
    ])
