"""
Simple Nav Isolated Test Launch
================================
Minimal bringup for testing odometry and SLAM-based waypoint following
without Nav2.  Starts:

  1. Robot State Publisher     — URDF -> /tf (base_link, imu_link, etc.)
  2. Phidgets Spatial          — raw IMU on /imu/data_raw
  3. IMU Relay (NED->ENU)      — corrects orientation frame -> /imu/data
  4. Wheel Odom Publisher      — integrates encoder ticks -> /wheel_odom
  5. EKF Local                 — fuses /wheel_odom + /imu/data -> /odometry/local
                                 publishes odom->base_link TF
  6. RPLidar                   — LaserScan on /scan_lower
  7. ZED open capture          — stereo depth camera on /dev/video2
  8. Pointcloud frame relay    — re-stamps ZED points with left_camera_link_optical
  9. Lane detection            — classical HSV -> /cv/lane_detections_cloud
 10. Depth detection           — ZED filtering -> /zed_node/left/obstacle_points
 11. Cartographer              — pure map builder: lidar + odom + point clouds
                                 publishes /map (occupancy grid) + map->odom TF (≈ identity,
                                 no SLAM corrections — just stamps scans at EKF position)
 12. GPS Covariance Relay      — /gps/fix -> /gps/fix_cov with covariance (always running;
                                 harmless if no GPS)
 13. Navsat Transform          — GPS + IMU + /odometry/local -> utm->map TF + /odometry/gps
                                 (publishes utm->map when GPS fix available)
 14. GPS Driver (optional)     — nmea_navsat_driver on serial port; enable with use_gps:=true
 15. RViz
 16. simple_nav_node           — open-loop relative goal follower using /odometry/local

TF chain:
  utm --(navsat_transform)--> map --(cartographer, ≈identity)--> odom --(ekf_local)--> base_link

Cartographer role: draws lidar returns onto the map at the EKF-reported position.
It does NOT do scan-based localization (optimize_every_n_nodes=0, occupied_space_weight=1e-9).
The map->odom TF it publishes is therefore ~identity — the real global anchor is navsat.

motor_control is NOT launched here — start it separately (systemd on the Pi, or
`ros2 run motor_control motor_control.py`).

Usage:
  ros2 launch simple_nav simple_nav.launch.py
  ros2 launch simple_nav simple_nav.launch.py lidar_port:=/dev/ttyUSB0
  ros2 launch simple_nav simple_nav.launch.py use_gps:=true gps_port:=/dev/ttyUSB1
  ros2 launch simple_nav simple_nav.launch.py magnetic_declination:=-0.24 use_gps:=true
  ros2 launch simple_nav simple_nav.launch.py log_level:=debug
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
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
        'log_level', default_value='info',
        description='Log level: debug, info, warn, error',
    )
    log_level = LaunchConfiguration('log_level')

    yaw_offset_arg = DeclareLaunchArgument(
        'yaw_offset', default_value='0.0',
        description='IMU heading calibration offset in DEGREES (ENU, about Up).',
    )
    yaw_offset = LaunchConfiguration('yaw_offset')

    orientation_stddev_arg = DeclareLaunchArgument(
        'orientation_stddev', default_value='0.05',
        description='IMU orientation covariance diagonal stddev (rad).',
    )
    orientation_stddev = LaunchConfiguration('orientation_stddev')

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/lidar_port_0',
        description='Serial port for the RPLidar (e.g. /dev/ttyUSB0).',
    )
    lidar_port = LaunchConfiguration('lidar_port')

    zed_device_arg = DeclareLaunchArgument(
        'zed_device', default_value='/dev/video2',
        description='Video device path for the ZED camera.',
    )
    zed_device = LaunchConfiguration('zed_device')

    use_gps_arg = DeclareLaunchArgument(
        'use_gps', default_value='false',
        description='Launch the NMEA GPS serial driver. Set true when GPS receiver is connected.',
    )
    use_gps = LaunchConfiguration('use_gps')

    gps_port_arg = DeclareLaunchArgument(
        'gps_port', default_value='/dev/gps_port_0',
        description='Serial port for the GPS receiver (e.g. /dev/ttyUSB1).',
    )
    gps_port = LaunchConfiguration('gps_port')

    gps_baud_arg = DeclareLaunchArgument(
        'gps_baud', default_value='4800',
        description='Baud rate for the GPS serial port.',
    )
    gps_baud = LaunchConfiguration('gps_baud')

    horizontal_stddev_arg = DeclareLaunchArgument(
        'horizontal_stddev', default_value='0.5',
        description='GPS horizontal stddev in metres (injected by gps_cov_relay).',
    )
    horizontal_stddev = LaunchConfiguration('horizontal_stddev')

    vertical_stddev_arg = DeclareLaunchArgument(
        'vertical_stddev', default_value='1.0',
        description='GPS vertical stddev in metres (injected by gps_cov_relay).',
    )
    vertical_stddev = LaunchConfiguration('vertical_stddev')

    magnetic_declination_arg = DeclareLaunchArgument(
        'magnetic_declination', default_value='0.0',
        description=(
            'Magnetic declination in radians for the test site. '
            'Toronto ≈ -0.24 rad. Passed to navsat_transform_node.'
        ),
    )
    magnetic_declination = LaunchConfiguration('magnetic_declination')

    # ── Robot description ─────────────────────────────────────────────────────

    urdf_path = PathJoinSubstitution([
        FindPackageShare('description'), 'rover_model', 'urdf', 'timbot.urdf.xacro',
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
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
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

    imu_relay = TimerAction(
        period=2.0,
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
        ],
    )

    # ── Wheel encoders ────────────────────────────────────────────────────────

    wheel_odom = Node(
        package='motor_odom',
        executable='odom_pub.py',
        name='wheel_odom_pub',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    # ── EKF Local ─────────────────────────────────────────────────────────────
    # Fuses /wheel_odom + /imu/data -> /odometry/local + odom->base_link TF.
    # This is the primary truth source for the entire stack.

    odom_yaml = PathJoinSubstitution([
        FindPackageShare('odom_state'), 'config', 'odom.yaml',
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
        parameters=[odom_yaml, {'use_sim_time': False}],
    )

    # ── RPLidar ───────────────────────────────────────────────────────────────

    rplidar = GroupAction(actions=[
        SetRemap(src='/scan', dst='/scan_lower'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('rplidar_ros'), '/launch/rplidar_a1_launch.py',
            ]),
            launch_arguments={
                'serial_port': lidar_port,
                'frame_id': 'bottom_lidar_link',
            }.items(),
        ),
    ])

    # ── ZED camera + CV pipeline ──────────────────────────────────────────────

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('timbot_launch'), '/launch/zed_open_capture.launch.py',
        ]),
        launch_arguments={'video_device': zed_device}.items(),
    )

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

    lane_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('lane_detection'), '/launch/launch.py',
        ]),
        launch_arguments={
            'sim': 'false',
            'lane_detection_mode': '1',
            'camera_width': '672',
            'camera_height': '376',
        }.items(),
    )

    depth_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('depth_detection'), '/launch/depth_detection.launch.py',
        ]),
        launch_arguments={
            'sim': 'false',
            'ramp_seg_using_lidar': 'false',
        }.items(),
    )

    # ── Cartographer: pure map builder ────────────────────────────────────────
    # Receives /scan_lower + /odometry/local + obstacle/lane point clouds.
    # optimize_every_n_nodes=0 means NO global SLAM, NO loop closure.
    # The Ceres scan matcher is pinned to odometry (occupied_space_weight=1e-9),
    # so scan data influences the map appearance only, not the pose estimate.
    # publish_to_tf=true: publishes map->odom, but ≈ identity since no corrections.
    # The real global anchor (utm->map) comes from navsat_transform below.

    carto_config_dir = PathJoinSubstitution([
        FindPackageShare('simple_nav'), 'config',
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

    # ── GPS pipeline ──────────────────────────────────────────────────────────
    # gps_cov_relay: always running; adds covariance to /gps/fix -> /gps/fix_cov.
    # navsat_transform: uses /odometry/local + GPS -> publishes utm->map TF.
    # GPS driver: conditional (use_gps:=true), launches nmea_navsat_driver.

    gps_cov_relay = Node(
        package='odom_state',
        executable='gps_cov_relay.py',
        name='gps_cov_relay',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'horizontal_stddev': horizontal_stddev,
            'vertical_stddev': vertical_stddev,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        respawn=True,
        remappings=[
            ('/odometry/filtered', '/odometry/local'),  # local EKF is the truth source
            ('/gps/fix', '/gps/fix_cov'),               # covariance-stamped GPS fix
            ('/imu', '/imu/data'),
        ],
        parameters=[
            odom_yaml,                                  # reads navsat_transform_node: section
            {'use_sim_time': False},
            {'magnetic_declination_radians': magnetic_declination},
            {'wait_for_datum': False},                  # use first GPS fix as origin
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    gps_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nmea_navsat_driver'), '/launch/nmea_serial_driver.launch.py',
        ]),
        launch_arguments={
            'serial_port': gps_port,
            'gps_baud': gps_baud,
        }.items(),
        condition=IfCondition(use_gps),
    )

    # ── RViz ──────────────────────────────────────────────────────────────────

    rviz_config = PathJoinSubstitution([
        FindPackageShare('simple_nav'), 'config', 'simple_nav.rviz',
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'warn'],
    )

    # ── Simple Nav: open-loop relative goal follower ──────────────────────────
    # Subscribes to /odometry/local (EKF local, odom frame) for pose feedback.
    # Cartographer's map->odom is ≈ identity so the nav targets are effectively
    # in the map frame too.

    simple_nav_yaml = PathJoinSubstitution([
        FindPackageShare('simple_nav'), 'config', 'simple_nav.yaml',
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
        use_gps_arg,
        gps_port_arg,
        gps_baud_arg,
        horizontal_stddev_arg,
        vertical_stddev_arg,
        magnetic_declination_arg,
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
        gps_cov_relay,
        navsat_transform,
        gps_driver,
        rviz,
        simple_nav,
    ])
