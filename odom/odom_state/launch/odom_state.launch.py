from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    position_covariance = LaunchConfiguration('position_covariance')
    orientation_covariance = LaunchConfiguration('orientation_covariance')
    horizontal_stddev = LaunchConfiguration('horizontal_stddev')
    vertical_stddev = LaunchConfiguration('vertical_stddev')
    
    # 1. Declare the use_sim_time argument (Default to true for safety in this context)
    use_sim_time_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('sim')

    # 2. Optional config_file argument — allows the main launch orchestrator
    #    to specify which YAML config to use (default: odom.yaml)
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='odom.yaml',
        description='Name of the odom config file in odom_state/config/'
    )

    # 3. only_errors argument — suppresses info-level log output
    only_errors_arg = DeclareLaunchArgument(
        'only_errors',
        default_value='false',
        description='If true, set log level to error instead of info'
    )
    only_errors = LaunchConfiguration('only_errors')
    log_level = PythonExpression([
        "'error' if '", only_errors, "' == 'true' else 'info'"
    ])

    # 3. Derive launch_state automatically
    # If use_sim_time is true, state is 'sim'. Otherwise 'real'.
    launch_state = PythonExpression([
        "'sim' if '", use_sim_time, "' == 'true' else 'real'"
    ])

    # 4. Path Setup — config file resolved via launch argument
    odom_yaml = PathJoinSubstitution([
        FindPackageShare('odom_state'), 'config', LaunchConfiguration('config_file')
    ])

    # 4. Nodes
    # Each node loads from the combined odom.yaml; node name matches the YAML key
    # so ROS2 automatically picks the correct section.
    
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        remappings=[('/odometry/filtered', '/odometry/local')],
        parameters=[
            odom_yaml, 
            {'use_sim_time': use_sim_time}, 
            {'launch_state': launch_state} 
        ]
    )

    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        remappings=[('/odometry/filtered', '/odometry/global')],
        parameters=[
            odom_yaml,
            {'use_sim_time': use_sim_time},
            {'launch_state': launch_state}
        ]
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        respawn=False,
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[('/odometry/filtered', '/odometry/global'), 
                    ('/gps/fix', '/gps/fix_cov'), 
                    ('/imu', '/imu/data')],
        parameters=[
            odom_yaml,
            {'use_sim_time': use_sim_time},
            {'launch_state': launch_state},
        ]
    )

    pose_relay = Node(
        package='odom_state',
        executable='pose_relay.py',
        name='pose_relay',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'input_topic': '/tracked_pose'},
            {'output_topic': '/tracked_pose_cov'},
            {'position_covariance': position_covariance},
            {'orientation_covariance': orientation_covariance}
        ]
    )

    gps_cov_relay = Node(
        package='odom_state',
        executable='gps_cov_relay.py',
        name='gps_cov_relay',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'horizontal_stddev': horizontal_stddev},
            {'vertical_stddev': vertical_stddev}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        only_errors_arg,
        ekf_local,
        pose_relay,
        gps_cov_relay,
        ekf_global,
        navsat_transform_node
    ])