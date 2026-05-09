"""
Robot Bringup Launch
====================
Launches the core robot infrastructure needed in BOTH sim and real modes:
  - Robot State Publisher  (URDF → /tf, /robot_description)
  - Joint State Publisher  (real only — in sim, Gazebo handles joint_states)
  - Twist Multiplexer      (cmd_vel routing from teleop/nav/etc.)

This is deliberately separate from spawn.launch.py so that real-rover
bringup can use it without pulling in the Gazebo bridge or spawn nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # --- Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('sim')

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error'
    )
    log_level = LaunchConfiguration('log_level')

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='false',
        description='Enable ZED camera sensor in URDF'
    )
    enable_camera = LaunchConfiguration('enable_camera')

    enable_lane_detection_arg = DeclareLaunchArgument(
        'enable_lane_detection',
        default_value='false',
        description='Enable lane detection related bringup nodes'
    )
    enable_lane_detection = LaunchConfiguration('enable_lane_detection')

    camera_fps_arg = DeclareLaunchArgument('camera_fps', default_value='5')
    camera_width_arg = DeclareLaunchArgument('camera_width', default_value='320')
    camera_height_arg = DeclareLaunchArgument('camera_height', default_value='180')
    camera_fps = LaunchConfiguration('camera_fps')
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')

    # --- Robot Description (URDF via xacro) ---
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

    # Robot State Publisher
    # Publishes /robot_description and /tf from URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content,
            'publish_frequency': 50.0,
        }]
    )

    # Joint State Publisher — real only
    # In sim, Gazebo publishes /joint_states via the bridge
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(use_sim_time),
        parameters=[{
            'rate': 50,
            'robot_description': robot_description_content,
        }]
    )

    # Twist Multiplexer
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('twist_mux'),
            '/launch/twist_mux_launch.py'
        ]),
        launch_arguments={
            'cmd_vel_out': 'cmd_vel',
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Point cloud frame relay for RViz in sim. Enabled only when lane detection is enabled.
    relay_output_frame = PythonExpression([
        "'left_camera_link' if '", use_sim_time, "' == 'true' else 'left_camera_link_optical'"
    ])

    pointcloud_relay = Node(
        package='description',
        executable='pointcloud_frame_relay.py',
        name='pointcloud_frame_relay',
        condition=IfCondition(enable_lane_detection),
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/zed_node/left/points',
            'output_topic': '/zed_node/left/points_rviz',
            'output_frame_id': relay_output_frame,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )

    # Point cloud filter — filters relay output for navigation
    pointcloud_rviz_filter = Node(
        name='pointcloud_rviz_filter',
        condition=IfCondition(enable_lane_detection),
        executable='/media/robotswithai/data1/utra/install/zed_camera_depth_cloud/lib/zed_camera_depth_cloud/pointcloud_filter_from_rviz.py',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        enable_camera_arg,
        enable_lane_detection_arg,
        camera_fps_arg, camera_width_arg, camera_height_arg,
        joint_state_publisher,
        robot_state_publisher,
        twist_mux,
        pointcloud_relay,
        pointcloud_rviz_filter,
    ])
