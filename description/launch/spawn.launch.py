from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition, IfCondition
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

    # log_level argument — controls verbosity (debug, info, warn, error)
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error'
    )
    log_level = LaunchConfiguration('log_level')

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

    # Camera enable argument — when true, the ZED camera sensor is included in the URDF
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='false',
        description='Enable ZED camera sensor in URDF and bridge camera topics'
    )
    enable_camera = LaunchConfiguration('enable_camera')

    # Camera settings
    camera_fps_arg = DeclareLaunchArgument('camera_fps', default_value='5')
    camera_width_arg = DeclareLaunchArgument('camera_width', default_value='320')
    camera_height_arg = DeclareLaunchArgument('camera_height', default_value='180')
    camera_fps = LaunchConfiguration('camera_fps')
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')

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
            ]),
            ' enable_camera:=', enable_camera,
            ' camera_fps:=', camera_fps,
            ' camera_width:=', camera_width,
            ' camera_height:=', camera_height,
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
            # Gazebo set pose, delete and creation of eentitiy to 
            '/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            '/world/default/delete@ros_gz_interfaces/srv/DeleteEntity',
            '/world/default/create@ros_gz_interfaces/srv/SpawnEntity',
            # Receive raw odometry FROM Gazebo (Optional, but good for debugging)
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '--ros-args', '--log-level', log_level,
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

    # 5. Camera Bridge — only launched when enable_camera is true
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        condition=IfCondition(enable_camera),
        arguments=[
            '/zed_node/left/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed_node/left/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed_node/left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/zed_node/left/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '--ros-args', '--log-level', log_level
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        x_arg, y_arg, z_arg, roll_arg, pitch_arg, yaw_arg,
        enable_camera_arg,
        camera_fps_arg, camera_width_arg, camera_height_arg,

        bridge,
        camera_bridge,
        spawn_robot,
        joint_state_publisher,
        robot_state_publisher,
        twist_mux,
    ])