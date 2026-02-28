from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    main_lidar_topic = LaunchConfiguration('main_lidar_topic')
    upper_lidar_topic = LaunchConfiguration('upper_lidar_topic')
    out_topic = LaunchConfiguration('out_topic')
    distance_to_second_lidar = LaunchConfiguration('distance_to_second_lidar')
    max_theta_degrees = LaunchConfiguration('max_theta_degrees')
    compare_lidar_time_tolerance_seconds = LaunchConfiguration('compare_lidar_time_tolerance_seconds')
    upper_lidar_start_index = LaunchConfiguration('upper_lidar_start_index')
    upper_lidar_stop_index = LaunchConfiguration('upper_lidar_stop_index')
    upper_lidar_angular_total_range = LaunchConfiguration('upper_lidar_angular_total_range')
    main_lidar_angular_total_range = LaunchConfiguration('main_lidar_angular_total_range')
    limit_output_range = LaunchConfiguration('limit_output_range')
    desired_output_total_range = LaunchConfiguration('desired_output_total_range')

    # --- Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('sim')

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
            'main_lidar_topic': main_lidar_topic,
            'upper_lidar_topic': upper_lidar_topic,
            'out_topic': out_topic,
            'distance_to_second_lidar': distance_to_second_lidar,
            'max_theta_degrees': max_theta_degrees,
            'compare_lidar_time_tolerance_seconds': compare_lidar_time_tolerance_seconds,
            'upper_lidar_start_index': upper_lidar_start_index,
            'upper_lidar_stop_index': upper_lidar_stop_index,
            'upper_lidar_angular_total_range': upper_lidar_angular_total_range,
            'main_lidar_angular_total_range': main_lidar_angular_total_range,
            'limit_output_range': limit_output_range,
            'desired_output_total_range': desired_output_total_range,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        only_errors_arg,
        dual_lidar_filter_node,
    ])
