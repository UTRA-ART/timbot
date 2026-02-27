from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # --- Arguments ---
    # We added use_sim_time here so we can pass it to everything
    use_sim_time_arg = DeclareLaunchArgument(
        'sim',
        default_value='true', # Default to true since we are spawning in sim
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

    # This is currently unused
    # world_type_arg = DeclareLaunchArgument(
    #     'world_type',
    #     default_value='pavement',
    #     description='World type for spawning robot'
    # )
    
    # Spawn Arguments
    x_arg = DeclareLaunchArgument('x', default_value='-19.5')
    y_arg = DeclareLaunchArgument('y', default_value='0')
    z_arg = DeclareLaunchArgument('z', default_value='0.05') # Lifted z to prevent jitter
    roll_arg = DeclareLaunchArgument('roll', default_value='0')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='1.5708')
    
    # --- Robot Description ---
    # Note: Ensure 'timbot.urdf.xacro' vs 'espresso.urdf.xacro' matches your actual file
    robot_description_content = ParameterValue(
        Command([
            'xacro ', 
            PathJoinSubstitution([
                FindPackageShare('description'),
                'rover_model',
                'urdf', 
                'timbot.urdf.xacro' 
            ])
        ]),
        value_type=str
    )
    
    # --- Nodes ---

    # 1. ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Bridge the Clock (so ROS knows the sim time)
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            
            # Bridge the Joint States (so ROS knows where the wheels are)
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            
            # Bridge the Lidar (Example: customize topic names as needed)
            '/scan_lower@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/scan_upper@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            
            # Bridge the GPS (gps_cov_relay adds covariance → /gps/fix_cov)
            '/gps/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
            
            # Bridge the IMU
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',

            # Allow ROS to send drive commands TO Gazebo (Note the ']')
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            
            # Receive raw odometry FROM Gazebo (Optional, but good for debugging)
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '--ros-args', '--log-level', log_level
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 2. Spawn Robot (The "Create" Node)
    # This reads the URDF from the topic published by Robot State Publisher below
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_timbot',
        arguments=[
            '-name', 'timbot',
            '-topic', 'robot_description', # Subscribes to the topic
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'), 
            '-z', LaunchConfiguration('z'),
            '-R', LaunchConfiguration('roll'),
            '-P', LaunchConfiguration('pitch'),
            '-Y', LaunchConfiguration('yaw')
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 3. Joint State Publisher
    # FIX: Only run this if use_sim_time is FALSE. 
    # In Sim, the Gazebo plugin handles this.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(use_sim_time), 
        parameters=[
            {'rate': 50,
            'robot_description': robot_description_content}
        ]
    )
    
    # 3. Robot State Publisher
    # Takes the joint states (from Gazebo or JSP) and publishes the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time,
            'robot_description': robot_description_content,
            'publish_frequency': 50.0}
        ]
    )
    
    # 4. Twist Multiplexer
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('twist_mux'),
            '/launch/twist_mux_launch.py'
        ]),
        launch_arguments={
            'cmd_vel_out': 'cmd_vel',
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        mute_warnings_arg,
        # world_type_arg,
        x_arg, y_arg, z_arg, roll_arg, pitch_arg, yaw_arg,
        
        bridge,
        spawn_robot,
        joint_state_publisher,
        robot_state_publisher,
        twist_mux,
    ])