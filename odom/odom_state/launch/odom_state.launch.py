from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    position_covariance = LaunchConfiguration('position_covariance')
    orientation_covariance = LaunchConfiguration('orientation_covariance')
    horizontal_stddev = LaunchConfiguration('horizontal_stddev')
    vertical_stddev = LaunchConfiguration('vertical_stddev')
    wait_for_datum = LaunchConfiguration('wait_for_datum')
    magnetic_declination_radians = LaunchConfiguration('magnetic_declination_radians')
    
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

    # 3. log_level argument — controls verbosity (debug, info, warn, error)
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error'
    )
    log_level = LaunchConfiguration('log_level')

    use_identity_map_odom_arg = DeclareLaunchArgument(
        'use_identity_map_odom',
        default_value='false',
        description='If true, publish static map->odom identity TF and skip ekf_global'
    )
    use_identity_map_odom = LaunchConfiguration('use_identity_map_odom')

    # GPS Covariance parameters
    horizontal_stddev_arg = DeclareLaunchArgument(
        'horizontal_stddev',
        default_value='0.5', # 0.5m accuracy for the Columbus P-7 Pro
        description='Horizontal standard deviation for GPS'
    )
    
    vertical_stddev_arg = DeclareLaunchArgument(
        'vertical_stddev',
        default_value='1.0', # Altitude is slightly noisier
        description='Vertical standard deviation for GPS'
    )

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
        remappings=[
            ('/odometry/filtered', '/odometry/local'),
            ('set_pose', '/ekf_local/set_pose'),
            ('/set_pose', '/ekf_local/set_pose')
        ],
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            odom_yaml, 
            {'use_sim_time': use_sim_time}, 
            {'launch_state': launch_state} 
        ]
    )

    # Global EKF owns map→odom and fuses RTAB /visual_odom (see odom.yaml).
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        remappings=[
            ('/odometry/filtered', '/odometry/global'),
            ('set_pose', '/ekf_global/set_pose'),
            ('/set_pose', '/ekf_global/set_pose')
        ],
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            odom_yaml,
            {'use_sim_time': use_sim_time},
            {'launch_state': launch_state}
        ],
        condition=IfCondition(PythonExpression(["'", use_identity_map_odom, "' == 'false'"]))
    )

    gps_static_transform = Node(
        package='odom_state',
        executable='gps_static_transform.py',
        name='gps_static_transform',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            odom_yaml,
            {'use_sim_time': use_sim_time},
            {'wait_for_datum': wait_for_datum},
            {'magnetic_declination_radians': magnetic_declination_radians},
            {'horizontal_stddev': horizontal_stddev},
            {'vertical_stddev': vertical_stddev}
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

    map_odom_identity_tf = Node(
        package='odom_state',
        executable='map_odom_identity_tf.py',
        name='map_odom_identity_tf',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(use_identity_map_odom)
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        horizontal_stddev_arg, # ADDED
        vertical_stddev_arg,   # ADDED
        log_level_arg,
        use_identity_map_odom_arg,
        ekf_local,
        pose_relay,
        gps_cov_relay,
        ekf_global,
        map_odom_identity_tf,
        gps_static_transform
    ])
