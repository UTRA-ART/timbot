"""
Simple Nav Isolated Test Launch
================================
Minimal bringup for testing odometry and SLAM-based waypoint following
without Nav2, GPS, cameras, or the full Timbot stack. Starts:

  1. Robot State Publisher     — URDF -> /tf (base_link, imu_link, etc.)
  2. Phidgets Spatial          — raw IMU on /imu/data_raw
  3. IMU Relay (NED->ENU)      — corrects orientation frame -> /imu/data
  4. Wheel Odom Publisher      — integrates encoder ticks -> /wheel_odom
  5. EKF Local                 — fuses /wheel_odom + /imu/data -> /odometry/local
  6. RPLidar                   — LaserScan on /scan_lower
  7. ZED open capture          — stereo depth camera on /dev/video2
                                 publishes /zed_node/left/{image,camera_info,depth_image,points}
  8. Pointcloud frame relay    — re-stamps /zed_node/left/points -> /zed_node/left/points_rviz
                                 with frame_id=left_camera_link_optical (real hardware)
  9. Lane detection            — classical HSV white-threshold -> /cv/lane_detections_cloud
 10. Depth detection           — ZED point cloud filtering -> /zed_node/left/obstacle_points
 11. Cartographer              — SLAM: scan + odom + IMU + obstacle_points + lane_cloud
                                 publishes /tracked_pose (map frame) + map->odom TF
 12. Pose Relay                — /tracked_pose -> /tracked_pose_cov
 13. simple_nav_node           — open-loop relative goal follower using /tracked_pose_cov

Data flow:
  wheel_odom + IMU -> ekf_local (/odometry/local) ──────────────────────────────┐
                                                                                 │
  RPLidar (/scan_lower) ─────────────────────────────────────────────────→ Cartographer
                                                                                 │
  ZED -> pointcloud_relay -> depth_detection -> /zed_node/left/obstacle_points ─┤
      -> lane_detection  -> /cv/lane_detections_cloud ───────────────────────────┘
                                                                                 │
                                                              /tracked_pose -> pose_relay
                                                                                 │
                                                                         /tracked_pose_cov
                                                                                 │
                                                                       simple_nav_node -> /cmd_vel

motor_control is NOT launched here — start it separately (systemd on the Pi, or
`ros2 run motor_control motor_control.py`). simple_nav publishes /cmd_vel directly.

Usage:
  ros2 launch simple_nav simple_nav.launch.py
  ros2 launch simple_nav simple_nav.launch.py lidar_port:=/dev/ttyUSB0 zed_device:=/dev/video2
  ros2 launch simple_nav simple_nav.launch.py yaw_offset:=5.0
  ros2 launch simple_nav simple_nav.launch.py log_level:=debug
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, Node, SetRemap
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error',
    )
    log_level = LaunchConfiguration('log_level')

    yaw_offset_arg = DeclareLaunchArgument(
        'yaw_offset',
        default_value='0.0',
        description='IMU heading calibration offset in DEGREES (ENU, about Up).',
    )
    yaw_offset = LaunchConfiguration('yaw_offset')

    orientation_stddev_arg = DeclareLaunchArgument(
        'orientation_stddev',
        default_value='0.05',
        description='IMU orientation covariance diagonal stddev (rad).',
    )
    orientation_stddev = LaunchConfiguration('orientation_stddev')

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/lidar_port_0',
        description='Serial port for the RPLidar (e.g. /dev/ttyUSB0).',
    )
    lidar_port = LaunchConfiguration('lidar_port')

    zed_device_arg = DeclareLaunchArgument(
        'zed_device',
        default_value='/dev/video2',
        description='Video device path for the ZED camera.',
    )
    zed_device = LaunchConfiguration('zed_device')

    # ── Robot description (URDF → /tf static frames) ─────────────────────────

    urdf_path = PathJoinSubstitution([
        FindPackageShare('description'),
        'rover_model',
        'urdf',
        'timbot.urdf.xacro',
    ])

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path, ' sim:=false']),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # ── IMU: Phidgets Spatial + NED->ENU relay ────────────────────────────────

    phidgets_container = ComposableNodeContainer(
        name='phidget_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='phidgets_spatial',
                plugin='phidgets::SpatialRosI',
                name='phidgets_spatial',
                parameters=[{
                    'use_orientation': True,
                    'spatial_algorithm': 'ahrs',
                    'frame_id': 'imu_link',
                    'data_interval_ms': 8,
                    'publish_rate': 0.0,
                    'use_sim_time': False,
                    'ahrs_angular_velocity_threshold': 1.0,
                    'ahrs_angular_velocity_delta_threshold': 0.1,
                    'ahrs_acceleration_threshold': 0.1,
                    'ahrs_mag_time': 10.0,
                    'ahrs_accel_time': 10.0,
                    'ahrs_bias_time': 1.25,
                }],
            ),
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )

    from launch.actions import TimerAction
    imu_relay = TimerAction(
        period=2.0,  # seconds
        actions=[
            Node(
                package='odom_state',
                executable='imu_relay.py',
                name='imu_relay',
                output='screen',
                parameters=[{
                    'input_topic': '/imu/data_raw',
                    'output_topic': '/imu/data',
                    'yaw_offset': yaw_offset,
                    'orientation_stddev': orientation_stddev,
                    'use_sim_time': False,
                }],
                arguments=['--ros-args', '--log-level', log_level],
            )
        ]
    )

    # ── Wheel encoders: odom publisher ───────────────────────────────────────
    # motor_control is started separately (e.g. via systemd on the Pi).
    # simple_nav publishes /cmd_vel directly; motor_control reads it there.

    # odom_pub integrates /left_wheel/ticks + /right_wheel/ticks
    # into /wheel_odom (nav_msgs/Odometry, odom → base_link).
    wheel_odom = Node(
        package='motor_odom',
        executable='odom_pub.py',
        name='wheel_odom_pub',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    # ── EKF Local: /wheel_odom + /imu/data → /odometry/local ─────────────────

    odom_yaml = PathJoinSubstitution([
        FindPackageShare('odom_state'),
        'config',
        'odom.yaml',
    ])

    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        remappings=[
            ('/odometry/filtered', '/odometry/local'),
            ('set_pose', '/ekf_local/set_pose'),
            ('/set_pose', '/ekf_local/set_pose'),
        ],
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            odom_yaml,
            {'use_sim_time': False},
        ],
    )

    # ── RPLidar → /scan_lower ─────────────────────────────────────────────────
    # rplidar_a1_launch.py publishes on /scan; SetRemap redirects it to
    # /scan_lower to match the topic name the rest of the stack expects.

    rplidar = GroupAction(actions=[
        SetRemap(src='/scan', dst='/scan_lower'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('rplidar_ros'),
                '/launch/rplidar_a1_launch.py',
            ]),
            launch_arguments={
                'serial_port': lidar_port,
                'frame_id': 'bottom_lidar_link',
            }.items(),
        ),
    ])

    # ── ZED camera + CV pipeline ──────────────────────────────────────────────

    # zed_open_capture_node: stereo USB feed → disparity → depth. Publishes
    # /zed_node/left/{image,camera_info,depth_image,points}.
    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('timbot_launch'),
            '/launch/zed_open_capture.launch.py',
        ]),
        launch_arguments={'video_device': zed_device}.items(),
    )

    # Re-stamps /zed_node/left/points with left_camera_link_optical so
    # depth_detection gets the correct frame on real hardware.
    pointcloud_relay = Node(
        package='description',
        executable='pointcloud_frame_relay.py',
        name='pointcloud_frame_relay',
        output='screen',
        parameters=[{
            'input_topic': '/zed_node/left/points',
            'output_topic': '/zed_node/left/points_rviz',
            'output_frame_id': 'left_camera_link_optical',
            'use_sim_time': False,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Lane detection (classical HSV, comp default) → /cv/lane_detections_cloud.
    lane_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('lane_detection'),
            '/launch/launch.py',
        ]),
        launch_arguments={
            'sim': 'false',
            'lane_detection_mode': '1',
            'camera_width': '672',
            'camera_height': '376',
        }.items(),
    )

    # Depth detection: filters /zed_node/left/points_rviz → /zed_node/left/obstacle_points.
    depth_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('depth_detection'),
            '/launch/depth_detection.launch.py',
        ]),
        launch_arguments={
            'sim': 'false',
            'ramp_seg_using_lidar': 'false',
        }.items(),
    )

    # ── Cartographer: SLAM using /scan_lower + /odometry/local + /imu/data ───
    # Uses cartographer_simple.lua (no point clouds, one laser scan).
    # Publishes /tracked_pose (PoseStamped, map frame) and map->odom TF.

    carto_config_dir = PathJoinSubstitution([
        FindPackageShare('simple_nav'),
        'config',
    ])

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', carto_config_dir,
            '-configuration_basename', 'cartographer_simple.lua',
            '--ros-args', '--log-level', log_level,
        ],
        remappings=[
            ('scan', '/scan_lower'),
            ('odom', '/odometry/local'),
            ('imu', '/imu/data'),
            ('points2_1', '/zed_node/left/obstacle_points'),
            ('points2_2', '/cv/lane_detections_cloud'),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0',
            '--ros-args', '--log-level', log_level,
        ],
    )

    # ── Pose Relay: /tracked_pose → /tracked_pose_cov ─────────────────────────
    # Cartographer publishes PoseStamped; ekf_global needs PoseWithCovarianceStamped.

    pose_relay = Node(
        package='odom_state',
        executable='pose_relay.py',
        name='pose_relay',
        output='screen',
        parameters=[{
            'input_topic': '/tracked_pose',
            'output_topic': '/tracked_pose_cov',
            'position_covariance': 0.05,
            'orientation_covariance': 0.01,
            'use_sim_time': False,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # ── Simple Nav: open-loop relative goal follower ──────────────────────────
    # Subscribes directly to /tracked_pose_cov (PoseWithCovarianceStamped, map frame).
    # ekf_global is not needed at this stage; add it once this is validated.

    simple_nav_yaml = PathJoinSubstitution([
        FindPackageShare('simple_nav'),
        'config',
        'simple_nav.yaml',
    ])

    simple_nav = Node(
        package='simple_nav',
        executable='simple_nav_node.py',
        name='simple_nav_node',
        output='screen',
        parameters=[simple_nav_yaml],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # ── Launch description ────────────────────────────────────────────────────

    return LaunchDescription([
        log_level_arg,
        yaw_offset_arg,
        orientation_stddev_arg,
        lidar_port_arg,
        zed_device_arg,
        robot_state_publisher,
        phidgets_container,
        imu_relay,
        wheel_odom,
        ekf_local,
        rplidar,
        zed,
        pointcloud_relay,
        lane_detection,
        depth_detection,
        cartographer_node,
        occupancy_grid_node,
        pose_relay,
        simple_nav,
    ])
