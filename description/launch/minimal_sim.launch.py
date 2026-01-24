"""
Minimal simulation launch file for testing Gazebo + RViz with Cartographer SLAM.
Uses existing configurations - same as real robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # ========== LAUNCH ARGUMENTS ==========
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Use Gazebo GUI'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='full',
        description='IGVC world: full, walls, ramp, plain'
    )
    
    world_type_arg = DeclareLaunchArgument(
        'world_type',
        default_value='pavement',
        description='World type: pavement'
    )
    
    # Spawn position arguments
    x_arg = DeclareLaunchArgument('x', default_value='-19.5')
    y_arg = DeclareLaunchArgument('y', default_value='0')
    z_arg = DeclareLaunchArgument('z', default_value='0.1')
    roll_arg = DeclareLaunchArgument('roll', default_value='0')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='1.5708')
    
    # ========== GAZEBO SETUP ==========
    gazebo_worlds_share = get_package_share_directory('gazebo_worlds')
    home_dir = os.path.expanduser('~')
    
    # Build world file path (world files are in share root, not pavement subfolder)
    world_file_path = os.path.join(gazebo_worlds_share, 'igvc_full.world')
    
    # Set up environment for Gazebo to find models (CRITICAL: include ~/.gazebo/models)
    env = os.environ.copy()
    env['GZ_SIM_RESOURCE_PATH'] = ':'.join([
        gazebo_worlds_share,
        os.path.join(home_dir, '.gazebo/models'),
        env.get('GZ_SIM_RESOURCE_PATH', '')
    ])
    
    # Launch Gazebo using ExecuteProcess (same as working load_igvc_full.launch.py)
    # -r flag auto-runs the simulation (unpaused)
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', 'sim', '-r', world_file_path, '--verbose'],
        output='screen',
        env=env
    )
    
    # ========== ROBOT DESCRIPTION ==========
    # Pass gazebo_version:=ignition to use Ignition Gazebo plugins
    robot_description_content = ParameterValue(
        Command([
            'xacro ', 
            PathJoinSubstitution([
                FindPackageShare('description'),
                'rover_model',
                'urdf', 
                'timbot.urdf.xacro'
            ]),
            ' gazebo_version:=ignition'
        ]),
        value_type=str
    )
    
    robot_description = {'robot_description': robot_description_content}
    
    # ========== SPAWN ROBOT IN GAZEBO ==========
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_timbot',
        arguments=[
            '-name', 'timbot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'), 
            '-z', LaunchConfiguration('z'),
            '-R', LaunchConfiguration('roll'),
            '-P', LaunchConfiguration('pitch'),
            '-Y', LaunchConfiguration('yaw')
        ],
        output='screen'
    )
    
    # ========== STATE PUBLISHERS (Required for TF/RViz) ==========
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'rate': 50}, robot_description]
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description]
    )

    # ========== ROS-IGNITION BRIDGE ==========
    # Bridge Ignition Gazebo topics to ROS 2
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Diff drive: cmd_vel (ROS -> Ign) and odom (Ign -> ROS)
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # Odometry from diff_drive for EKF
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            # IMU sensor - publish to /imu/data for EKF (matches odom_local.yaml)
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            # GPS/NavSat sensor - publish to /gps/fix for navsat_transform
            '/gps/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
            # LIDARs
            '/scan_lower@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/scan_upper@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen'
    )
    
    # Relay /odom to /wheel_odom/quat_synced for EKF (expected by odom_local.yaml)
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        arguments=['/odom', '/wheel_odom/quat_synced'],
        output='screen'
    )
    
    # ========== TOPIC RELAY: scan_lower -> scan_modified ==========
    # The nav2_params.yaml expects /scan_modified (processed by scheduler in real robot)
    # In simulation, we relay /scan_lower directly to /scan_modified
    scan_relay = Node(
        package='topic_tools',
        executable='relay',
        name='scan_relay',
        arguments=['/scan_lower', '/scan_modified'],
        output='screen'
    )

    # ========== TOPIC RELAY: nav_vel -> cmd_vel ==========
    # Nav2 controller outputs to /nav_vel, but Gazebo diff_drive expects /cmd_vel
    nav_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='nav_vel_relay',
        arguments=['/nav_vel', '/cmd_vel'],
        output='screen'
    )

    # ========== RVIZ ==========
    rviz_config = PathJoinSubstitution([
        FindPackageShare("description"),
        "rviz",
        "timbot.rviz"
    ])
    
    # Delay RViz launch to ensure robot_description is available
    rviz_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                condition=IfCondition(LaunchConfiguration('rviz')),
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=["-d", rviz_config],
                output="screen"
            )
        ]
    )
    
    # ========== CARTOGRAPHER SLAM ==========
    # Uses existing cartographer.lua config
    # Provides: map frame, map->odom transform
    cartographer_config_dir = PathJoinSubstitution([
        FindPackageShare('description'),
        'config'
    ])
    
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'cartographer.lua'
        ],
        remappings=[
            ('scan', '/scan_lower'),  # Bottom LIDAR for Cartographer
            ('scan_2', '/scan_upper'),  # Top LIDAR (if cv detections available, this becomes 2nd scan)
            ('imu', '/imu/data'),  # IMU topic from simulation
            ('fix', '/gps/fix'),  # GPS topic from simulation
        ]
    )
    
    # Cartographer occupancy grid node (publishes /map for Nav2)
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0'
        ]
    )
    
    # ========== ODOM (robot_localization) ==========
    # NOTE: odom_state.launch.py doesn't support use_sim_time arg, so we launch EKF nodes directly
    odom_state_dir = get_package_share_directory('odom_state')
    odom_local_yaml = os.path.join(odom_state_dir, 'config', 'odom_local.yaml')
    odom_global_yaml = os.path.join(odom_state_dir, 'config', 'odom_global.yaml')
    navsat_yaml = os.path.join(odom_state_dir, 'config', 'navsat.yaml')

    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        remappings=[("odometry/filtered", "odometry/local")],
        parameters=[odom_local_yaml, {
            'use_sim_time': True,
            'publish_tf': True,  # Publish odom->base_link transform (required for Cartographer)
        }]
    )

    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        remappings=[('odometry/filtered', 'odometry/global')],
        parameters=[odom_global_yaml, {'use_sim_time': True}]
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        respawn=True,
        remappings=[('odometry/filtered', 'odometry/local')],
        parameters=[navsat_yaml, {'use_sim_time': True}]
    )
    
    # ========== NAV2 (move_base) ==========
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav_stack'),
                'launch',
                'move_base.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        # Arguments
        use_gui_arg,
        rviz_arg,
        world_arg,
        world_type_arg,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        # Gazebo
        gazebo,
        # Robot
        spawn_robot,
        joint_state_publisher,
        robot_state_publisher,
        # ROS-Ignition Bridge (bridges sensor data from Gazebo to ROS)
        ros_gz_bridge,
        # Odom relay (odom -> wheel_odom/quat_synced for EKF)
        odom_relay,
        # Scan relay (scan_lower -> scan_modified for nav2)
        scan_relay,
        # Nav vel relay (nav_vel -> cmd_vel for Gazebo)
        nav_vel_relay,
        # Odom (robot_localization EKF nodes with use_sim_time)
        ekf_local,
        ekf_global,
        navsat_transform_node,
        # Cartographer SLAM (publishes map->odom and /map)
        TimerAction(
            period=5.0,  # Delay to let sensors start publishing
            actions=[cartographer_node, occupancy_grid_node]
        ),
        # Nav2
        nav_launch,
        # RViz
        rviz_node
    ])
