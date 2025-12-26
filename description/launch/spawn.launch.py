from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EqualsSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Declare launch arguments
    launch_state_arg = DeclareLaunchArgument(
        'launch_state',
        default_value='sim',
        description='Launch state parameter'
    )
    
    world_type_arg = DeclareLaunchArgument(
        'world_type',
        default_value='pavement',
        description='World type for spawning robot'
    )
    
    # Conditional spawn positions based on world_type
    # For pavement world type - course start position
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='-19.5',
        description='X position'
    )
    
    y_arg = DeclareLaunchArgument(
        'y', 
        default_value='0',
        description='Y position'
    )
    
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.0026', 
        description='Z position'
    )
    
    roll_arg = DeclareLaunchArgument(
        'roll',
        default_value='0',
        description='Roll orientation'
    )
    
    pitch_arg = DeclareLaunchArgument(
        'pitch',
        default_value='0',
        description='Pitch orientation'
    )
    
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='1.5708',
        description='Yaw orientation'
    )
    
    # Parse URDF with xacro
    robot_description_content = ParameterValue(
        Command([
            'xacro ', 
            PathJoinSubstitution([
                FindPackageShare('description'),
                'rover_model',
                'urdf', 
                'espresso.urdf.xacro'
            ])
        ]),
        value_type=str
    )
    
    robot_description = {'robot_description': robot_description_content}
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_espresso',
        arguments=[
            '-entity', 'espresso',
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
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'rate': 50},
            robot_description
        ]
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description]
    )
    
    # Twist multiplexer
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('twist_mux'),
            '/launch/twist_mux.launch.py'
        ]),
        launch_arguments={
            'cmd_vel_out': 'cmd_vel'
        }.items()
    )
    
    # ZED camera emulation
    zed_emulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('description'),
            '/launch/zed_emulation.launch.py'
        ]),
        launch_arguments={
            'camera_ns': 'zed_node'
        }.items()
    )
    
    # Odometry computation
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('odom'),
            '/launch/odom.launch.py'
        ]),
        launch_arguments={
            'launch_state': 'sim'
        }.items()
    )

    return LaunchDescription([
        launch_state_arg,
        world_type_arg,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        spawn_robot,
        joint_state_publisher,
        robot_state_publisher,
        twist_mux,
        zed_emulation,
        odom_launch
    ])
