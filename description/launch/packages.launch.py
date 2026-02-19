from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    # --- Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('sim')

    # --- 1. Localization (Odom State) ---
    odom_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('odom_state'),
            '/launch/odom_state.launch.py'
        ]),
        launch_arguments={
            'sim': use_sim_time,
        }.items()
    )

    # --- 2. Vision (ZED Camera) [FUTURE] ---
    # zed_wrapper_launch = IncludeLaunchDescription(...)

    # --- 3. SLAM (Cartographer) ---
    cartographer_config_dir = PathJoinSubstitution([
        FindPackageShare('description'),
        'config',
    ])

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'cartographer.lua',
            '--ros-args', '--log-level', 'warn'
        ],
        remappings=[
            ('scan', '/scan_modified'),  # Bottom LIDAR after filter_lidar_data node
            ('imu', '/imu/data'),  # IMU topic from simulation
            # GPS disabled in cartographer — navsat handles GPS separately
        ]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0',
            '--ros-args', '--log-level', 'warn'
        ]
    )

    # --- 4. Navigation (Nav2) [FUTURE] ---
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav_stack'),
                'launch',
                'move_base.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # filter_lidar_data launch
    filter_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('filter_lidar_data'),
            '/launch/filter_lidar_data.launch.py'
        ]),
        launch_arguments={
            'sim': use_sim_time,
        }.items()
    )

    # load_waypoints launch
    load_waypoints_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('load_waypoints'),
            '/launch/load_waypoints.launch.py'
        ]),
        launch_arguments={
            'sim': use_sim_time,
        }.items()
    )

    # --- 5. Rviz (Visualization) ---
    rviz_config = PathJoinSubstitution([
        FindPackageShare("description"),
        "rviz",
        "timbot.rviz"
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config, "--ros-args", "--log-level", "warn"],
        parameters=[{'use_sim_time': use_sim_time}],
        output="screen"
    )

    return LaunchDescription([
        use_sim_time_arg,
        odom_state_launch,
        cartographer_node,
        occupancy_grid_node,
        filter_lidar_launch,
        rviz_node,
        TimerAction(period=2.0, actions=[nav2_launch]),
        TimerAction(period=4.0, actions=[load_waypoints_launch]),
    ])