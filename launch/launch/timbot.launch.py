"""
Timbot Central Launch Orchestrator
===================================
Launches all modules sequentially based on a YAML config file.
Supports both event-driven topic checking and static timer delays.

Usage:
  ros2 launch timbot_launch timbot.launch.py config:=sim.yaml
"""

import os
import yaml
from launch.actions import SetEnvironmentVariable
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
    LogInfo,
    EmitEvent,
    TimerAction,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.events import Shutdown as ShutdownEvent
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


# =============================================================================
# Stage Launchers
# =============================================================================

def launch_gazebo(config: dict, sim: bool, context: LaunchContext) -> list:
    gazebo_cfg = config.get('gazebo', {})
    gui = str(gazebo_cfg.get('gui', True)).lower()
    world_file = str(gazebo_cfg.get('world_file', 'track.world'))
    log_level = gazebo_cfg.get('log_level', 'info')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_worlds'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'gui': gui,
            'world_file': world_file,
            'log_level': log_level,
        }.items()
    )
    return [gazebo_launch]


def launch_robot_bringup(config: dict, sim: bool, context: LaunchContext) -> list:
    """Robot State Publisher + Joint State Publisher + Twist Mux.

    Must run before Spawn (sim) or Odom State (real) so that
    /robot_description and /tf are available.
    """
    bringup_cfg = config.get('robot_bringup', {})
    log_level = bringup_cfg.get('log_level', 'info')
    cv_cfg = config.get('lane_detection', {})
    cv_enabled = cv_cfg.get('enabled', False)
    enable_camera = str(cv_enabled).lower()

    launch_args = {
        'sim': str(sim).lower(),
        'log_level': log_level,
        'enable_camera': enable_camera,
        'enable_lane_detection': enable_camera,
        'camera_fps': str(cv_cfg.get('camera_fps', 5)),
        'camera_width': str(cv_cfg.get('camera_width', 320)),
        'camera_height': str(cv_cfg.get('camera_height', 180)),
    }

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('description'),
            '/launch/robot_bringup.launch.py'
        ]),
        launch_arguments=launch_args.items()
    )
    return [bringup_launch]


def launch_spawn(config: dict, sim: bool, context: LaunchContext) -> list:
    """Gazebo bridge + spawn entity.  Sim-only."""
    spawn_cfg = config.get('spawn', {})
    log_level = spawn_cfg.get('log_level', 'info')
    cv_cfg = config.get('lane_detection', {})
    cv_enabled = cv_cfg.get('enabled', False)
    enable_camera = str(cv_enabled).lower()

    launch_args = {
        'sim': str(sim).lower(),
        'log_level': log_level,
        'x': str(spawn_cfg.get('x', '-19.5')),
        'y': str(spawn_cfg.get('y', '0')),
        'z': str(spawn_cfg.get('z', '0.05')),
        'roll': str(spawn_cfg.get('roll', '0')),
        'pitch': str(spawn_cfg.get('pitch', '0')),
        'yaw': str(spawn_cfg.get('yaw', '1.5708')),
        'enable_camera': enable_camera,
    }

    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('description'),
            '/launch/spawn.launch.py'
        ]),
        launch_arguments=launch_args.items()
    )
    return [spawn_launch]


def launch_odom_state(config: dict, sim: bool, context: LaunchContext) -> list:
    odom_cfg = config.get('odom_state', {})
    log_level = odom_cfg.get('log_level', 'info')
    datum_lat = odom_cfg.get('datum', [0.0, 0.0, 0.0])[0]
    datum_lon = odom_cfg.get('datum', [0.0, 0.0, 0.0])[1]
    datum_alt = odom_cfg.get('datum', [0.0, 0.0, 0.0])[2]   
    launch_args = {
        'sim': str(sim).lower(),
        'log_level': log_level,
        'position_covariance': str(odom_cfg.get('position_covariance', 0.05)),
        'orientation_covariance': str(odom_cfg.get('orientation_covariance', 0.01)),
        'horizontal_stddev': str(odom_cfg.get('horizontal_stddev', 3.0)),
        'vertical_stddev': str(odom_cfg.get('vertical_stddev', 4.0)),
        'wait_for_datum': str(odom_cfg.get('wait_for_datum', False)).lower(),
        'datum': f'[{datum_lat}, {datum_lon}, {datum_alt}]',
        'magnetic_declination_radians': str(odom_cfg.get('magnetic_declination_radians', 0.0))
    }
    config_file = odom_cfg.get('config_file', '')
    if config_file:
        launch_args['config_file'] = config_file
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('odom_state'),
            '/launch/odom_state.launch.py'
        ]),
        launch_arguments=launch_args.items()
    )
    return [odom_launch]

def launch_filter_lidar(config: dict, sim: bool, context: LaunchContext) -> list:
    lidar_cfg = config.get('filter_lidar', {})
    log_level = lidar_cfg.get('log_level', 'info')
    ramp_seg_using_lidar = config.get('ramp_seg_using_lidar', True)
    filter_args = {
        'sim': str(sim).lower(),
        'log_level': log_level,
        'main_lidar_topic': str(lidar_cfg.get('main_lidar_topic', '/scan_lower')),
        'upper_lidar_topic': str(lidar_cfg.get('upper_lidar_topic', '/scan_upper')),
        'out_topic': str(lidar_cfg.get('out_topic', '/scan_modified')),
        'distance_to_second_lidar': str(lidar_cfg.get('distance_to_second_lidar', 0.14)),
        'max_theta_degrees': str(lidar_cfg.get('max_theta_degrees', 20.0)),
        'compare_lidar_time_tolerance_seconds': str(lidar_cfg.get('compare_lidar_time_tolerance_seconds', 2)),
        'upper_lidar_start_index': str(lidar_cfg.get('upper_lidar_start_index', 0)),
        'upper_lidar_stop_index': str(lidar_cfg.get('upper_lidar_stop_index', 1080)),
        'upper_lidar_angular_total_range': str(lidar_cfg.get('upper_lidar_angular_total_range', 360)),
        'main_lidar_angular_total_range': str(lidar_cfg.get('main_lidar_angular_total_range', 360)),
        'limit_output_range': str(lidar_cfg.get('limit_output_range', True)).lower(),
        'desired_output_total_range': str(lidar_cfg.get('desired_output_total_range', 180)),
        'ramp_seg_using_lidar': str(ramp_seg_using_lidar).lower(),
    }
    filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('filter_lidar_data'),
            '/launch/filter_lidar_data.launch.py'
        ]),
        launch_arguments=filter_args.items()
    )
    return [filter_launch]


def launch_lane_detection(config: dict, sim: bool, context: LaunchContext) -> list:
    cv_cfg = config.get('lane_detection', {})
    log_level = cv_cfg.get('log_level', 'info')
    lane_detection_mode = str(cv_cfg.get('lane_detection_mode', 0))

    cv_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('lane_detection'),
            '/launch/launch.py'
        ]),
        launch_arguments={
            'sim': str(sim).lower(),
            'log_level': log_level,
            'lane_detection_mode': lane_detection_mode,
            'camera_width': str(cv_cfg.get('camera_width', 640)),
            'camera_height': str(cv_cfg.get('camera_height', 320)),
            'white_sensitivity': str(cv_cfg.get('white_sensitivity', 20)),
            'downscale_factor':  str(cv_cfg.get('downscale_factor', 1)),
            'horizon_crop':      str(cv_cfg.get('horizon_crop', 0.15)),
            'morph_size':        str(cv_cfg.get('morph_size', 3)),
            'morph_open_iters':  str(cv_cfg.get('morph_open_iters', 1)),
            'morph_close_iters': str(cv_cfg.get('morph_close_iters', 1)),
        }.items()
    )
    return [cv_launch]


def launch_depth_detection(config: dict, sim: bool, context: LaunchContext) -> list:
    depth_cfg = config.get('depth_detection', {})
    log_level = depth_cfg.get('log_level', 'info')
    ramp_seg_using_lidar = config.get('ramp_seg_using_lidar', True)

    # depth_detection subscribes to /zed_node/left/points_rviz, which is only
    # published by pointcloud_relay in robot_bringup when lane_detection is enabled.
    if not config.get('lane_detection', {}).get('enabled', False):
        print('[timbot_launch] depth_detection skipped: lane_detection is disabled (no points_rviz feed)', flush=True)
        return []

    depth_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('depth_detection'),
            '/launch/depth_detection.launch.py'
        ]),
        launch_arguments={
            'sim': str(sim).lower(),
            'log_level': log_level,
            'config_file': depth_cfg.get('config_file', 'depth_detection.yaml'),
            'ramp_seg_using_lidar': str(ramp_seg_using_lidar).lower(),
        }.items()
    )

    return [depth_launch]

def launch_cartographer(config: dict, sim: bool, context: LaunchContext) -> list:
    carto_cfg = config.get('cartographer', {})
    log_level = carto_cfg.get('log_level', 'info')
    carto_config_file = carto_cfg.get('config_file', 'cartographer.lua')

    cartographer_config_dir = PathJoinSubstitution([
        FindPackageShare('description'),
        'config',
    ])

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': sim}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', carto_config_file,
            '--ros-args', '--log-level', log_level
        ],
        remappings=[
            ('scan', '/scan_modified'),
            ('points2_1', '/zed_node/left/obstacle_points'),
            ('points2_2', '/cv/lane_detections_cloud'),
            ('odom', '/odometry/local'),
            ('imu', '/imu/data'),
            ('fix', '/gps/fix_cov'),
        ]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': sim}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0',
            '--ros-args', '--log-level', log_level
        ]
    )

    return [cartographer_node, occupancy_grid_node]


def launch_rviz(config: dict, sim: bool, context: LaunchContext) -> list:
    rviz_cfg = config.get('rviz', {})
    rviz_config_file = rviz_cfg.get('rviz_config', 'timbot.rviz')
    rviz_config = PathJoinSubstitution([
        FindPackageShare("description"),
        "rviz",
        rviz_config_file
    ])
    log_level = rviz_cfg.get('log_level', 'info')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config, "--ros-args", "--log-level", log_level],
        parameters=[{'use_sim_time': sim}],
        output="screen"
    )
    return [rviz_node]


def launch_nav_stack(config: dict, sim: bool, context: LaunchContext) -> list:
    nav_cfg = config.get('nav_stack', {})
    log_level = nav_cfg.get('log_level', 'info')

    launch_args = {
        'sim': str(sim).lower(),
        'log_level': log_level,
    }

    config_file = nav_cfg.get('config_file', '')
    if config_file:
        launch_args['config_file'] = config_file

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav_stack'),
            '/launch/move_base.launch.py'
        ]),
        launch_arguments=launch_args.items()
    )
    return [nav_launch]


def launch_load_waypoints(config: dict, sim: bool, context: LaunchContext) -> list:
    wp_cfg = config.get('load_waypoints', {})
    log_level = wp_cfg.get('log_level', 'info')

    launch_args = {
        'sim': str(sim).lower(),
        'log_level': log_level,
    }

    config_file = wp_cfg.get('config_file', '')
    if config_file:
        launch_args['config_file'] = config_file

    wp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('load_waypoints'),
            '/launch/load_waypoints.launch.py'
        ]),
        launch_arguments=launch_args.items()
    )
    return [wp_launch]


# =============================================================================
# Hardware Driver Stages (real rover only, sim=False)
# =============================================================================
# Each entry: (display_name, launcher_fn, expected_topics, delay_sec)
# launcher_fn signature: (context: LaunchContext) -> list[Action]
# expected_topics: list of ROS topic names that must appear before
#                  the next driver is started (empty → use delay_sec).
# =============================================================================

def _hw_driver(pkg: str, launch_file: str, extra_args: dict | None = None,
               remappings: list[tuple[str, str]] | None = None):
    """Return a launcher function that wraps a ros2 IncludeLaunchDescription.

    Parameters
    ----------
    remappings : list of (src, dst) pairs, optional
        Topic remappings applied via SetRemap inside a GroupAction so
        they take effect for every node in the included launch file.
    """
    def launcher(context: LaunchContext) -> list:
        args = extra_args or {}
        include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare(pkg),
                f'/launch/{launch_file}',
            ]),
            launch_arguments=args.items(),
        )
        if remappings:
            remap_actions = [SetRemap(src=src, dst=dst) for src, dst in remappings]
            return [GroupAction(actions=remap_actions + [include])]
        return [include]
    return launcher


def _exec_driver(cmd: list[str], cwd: str | None = None, name: str | None = None):
    """Return a launcher function that wraps an ExecuteProcess command."""
    def launcher(context: LaunchContext) -> list:
        return [ExecuteProcess(cmd=cmd, cwd=cwd, name=name, output='screen')]
    return launcher


def _normalize_exec_cmd(cmd_value: object, default_cmd: list[str]) -> list[str]:
    """Normalize a YAML command entry into ExecuteProcess cmd form."""
    if isinstance(cmd_value, list) and cmd_value:
        return cmd_value
    if isinstance(cmd_value, str) and cmd_value.strip():
        return ['bash', '-lc', cmd_value]
    return default_cmd


# ---------------------------------------------------------------------------
# Driver definitions
# (name, launcher_fn, expected_topics, delay_sec)
# ---------------------------------------------------------------------------
def build_hardware_driver_stages(config: dict) -> list:
    """Build the hardware driver stage list from the loaded config.

    Ports and other per-environment settings are read from the config dict
    so they can be set in the YAML (e.g. comp.yaml) without touching this file.
    """
    gps_port        = config.get('gps_port',        '/dev/ttyUSB0')
    gps_baud = str(config.get('gps_baud', '57600'))
    lidar_lower_port = config.get('lidar_lower_port', '/dev/ttyUSB3')
    # lidar_upper_port = config.get('lidar_upper_port', '/dev/ttyUSB4')
    team_laptop = config.get('team_laptop', False)
    refresh_motors_cmd = [
    'bash', 
    '-c', 
    'echo "Kicking ROS 2 Daemon..."; ros2 daemon stop && ros2 daemon start; echo "Scanning for topics..." && ros2 topic list'
]


    zed_cfg = config.get('zed_camera', {})
    zed_expected_topics = zed_cfg.get('expected_topics', ['/zed_node/left/image'])
    zed_delay_sec = float(zed_cfg.get('delay_sec', 8.0))
    zed_video_device = zed_cfg.get('video_device', '/dev/video3')
    zed_auto_exposure = str(zed_cfg.get('auto_exposure', True)).lower()
    zed_exposure = str(zed_cfg.get('exposure', 50))
    zed_gain = str(zed_cfg.get('gain', 50))
    zed_gamma = str(zed_cfg.get('gamma', 5))
    zed_num_disparities = str(zed_cfg.get('num_disparities', 96))
    zed_block_size = str(zed_cfg.get('block_size', 3))
    zed_p1_multiplier = str(zed_cfg.get('p1_multiplier', 8))
    zed_p2_multiplier = str(zed_cfg.get('p2_multiplier', 32))

    zed_stage = (
        'Driver: ZED Open Capture',
        _hw_driver('timbot_launch', 'zed_open_capture.launch.py', {
            'video_device': zed_video_device,
            'auto_exposure': zed_auto_exposure,
            'exposure': zed_exposure,
            'gain': zed_gain,
            'gamma': zed_gamma,
            'num_disparities': zed_num_disparities,
            'block_size': zed_block_size,
            'p1_multiplier': zed_p1_multiplier,
            'p2_multiplier': zed_p2_multiplier,
        }),
        zed_expected_topics,
        zed_delay_sec,
    )
    driver_stages = [
        (
            'Running refresh_motor command',
            _exec_driver(cmd=refresh_motors_cmd, name='refresh_motors'),
            [],
            2.0,
        ),
        # (
        #     'Driver: GPS',
        #     _hw_driver('nmea_navsat_driver', 'nmea_serial_driver.launch.py', {
        #         'serial_port': gps_port,
        #         'gps_baud': gps_baud,
        #     }, remappings=[('/fix', '/gps/fix')]),
        #     ['/gps/fix'],
        #     5.0,
        # ),
        (
            'Driver: IMU',
            # Phidgets (raw) + imu_filter_madgwick → ENU orientation on /imu/data.
            # The onboard AHRS is NOT used (its quaternion is NED); madgwick fuses
            # raw accel/gyro/mag directly in ENU. See imu_bringup.launch.py.
            _hw_driver('timbot_launch', 'imu_bringup.launch.py', {
                'sim': 'false',
            }),
            ['/imu/data'],
            8.0,
        ),
        (
            'Driver: LiDAR Lower',
            _hw_driver('rplidar_ros', 'rplidar_a1_launch.py', {
                'serial_port': lidar_lower_port,
                'frame_id': 'bottom_lidar_link',
            }, remappings=[('/scan', '/scan_lower')]),
            ['/scan_lower'],
            5.0,
        ),
        # (
        #     'Driver: LiDAR Upper',
        #     _hw_driver('rplidar_ros', 'rplidar_a1_launch.py', {
        #         'serial_port': lidar_upper_port,
        #         'frame_id': 'top_lidar_link',
        #     }, remappings=[('/scan', '/scan_upper')]),
        #     ['/scan_upper'],
        #     5.0,
        # ),
        zed_stage,
        (
            'Driver: RPi Sync',
            _hw_driver('motor_odom', 'odom_pub.launch.py'),
            ['/wheel_odom'],
            3.0,
        ),
    ]

    return driver_stages


# =============================================================================
# Sequential Event-Driven Orchestrator
# =============================================================================

LAUNCH_STAGES = [
    ('Gazebo',         'gazebo',         launch_gazebo),
    ('Spawn',          'spawn',          launch_spawn),
    ('Robot Bringup',  'robot_bringup',  launch_robot_bringup),
    ('Odom State',     'odom_state',     launch_odom_state),
    ('Filter Lidar',   'filter_lidar',   launch_filter_lidar),
    ('Lane Detection', 'lane_detection', launch_lane_detection),
    ('Depth Detection','depth_detection',launch_depth_detection),
    ('Cartographer',   'cartographer',   launch_cartographer),
    ('RViz',           'rviz',           launch_rviz),
    ('Nav Stack',      'nav_stack',      launch_nav_stack),
    ('Load Waypoints', 'load_waypoints', launch_load_waypoints),
]

def _make_topic_waiter(stage_name: str, topics: list, waiter_id: str) -> ExecuteProcess:
    """Build a shell process that polls ros2 topic list until all topics appear."""
    topics_str = " ".join(topics)
    wait_cmd = (
        f"echo '[timbot_launch] Waiting for {stage_name} topics: {topics_str}...'; "
        f"while true; do "
        f"  LIST=$(ros2 topic list); "
        f"  ALL_FOUND=true; "
        f"  for t in {topics_str}; do "
        f"    if ! echo \"$LIST\" | grep -q \"^$t$\"; then "
        f"      ALL_FOUND=false; "
        f"      break; "
        f"    fi; "
        f"  done; "
        f"  if [ \"$ALL_FOUND\" = true ]; then "
        f"    echo '[timbot_launch] {stage_name} topics confirmed! Proceeding...'; "
        f"    exit 0; "
        f"  fi; "
        f"  sleep 1.0; "
        f"done"
    )
    return ExecuteProcess(
        cmd=['sh', '-c', wait_cmd],
        name=f'waiter_{waiter_id}',
        output='screen',
    )


def build_driver_chain(driver_stages, current_index, use_topic_check, next_pipeline_actions):
    """Recursively build a sequential chain of hardware-driver launch actions.

    Each driver stage: launch the driver, then either wait for its expected
    topics (topic-check mode) or wait a fixed delay before starting the next
    driver.  After all drivers finish the *next_pipeline_actions* are triggered.
    """
    if current_index >= len(driver_stages):
        # All drivers launched — hand off to the main pipeline.
        return next_pipeline_actions

    stage_name, launcher_fn, expected_topics, delay_sec = driver_stages[current_index]

    # Tail of the chain (everything after this driver).
    tail_actions = build_driver_chain(
        driver_stages, current_index + 1, use_topic_check, next_pipeline_actions
    )

    try:
        current_actions = launcher_fn(None)   # context not needed for IncludeLaunch
    except Exception as e:
        print(f"[timbot_launch] ERROR building {stage_name}: {e}", flush=True)
        return [EmitEvent(event=ShutdownEvent(reason=f"Failed to build {stage_name}"))]

    current_actions.insert(0, LogInfo(msg=f'[timbot_launch] LAUNCHING: {stage_name}'))

    if not tail_actions:
        return current_actions

    waiter_id = stage_name.lower().replace(' ', '_').replace(':', '')

    if use_topic_check and expected_topics:
        waiter_proc = _make_topic_waiter(stage_name, expected_topics, waiter_id)
        event_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=waiter_proc,
                on_exit=tail_actions,
            )
        )
        current_actions.append(waiter_proc)
        current_actions.append(event_handler)
    else:
        current_actions.append(TimerAction(period=delay_sec, actions=tail_actions))

    return current_actions


def build_stage_chain(stages_info, current_index, config, sim, context):
    if current_index >= len(stages_info):
        return []

    stage_name, config_key, launcher_fn = stages_info[current_index]
    stage_cfg = config.get(config_key, {})
    use_topic_check = config.get('use_topic_check', True)

    try:
        current_actions = launcher_fn(config, sim, context)
    except Exception as e:
        print(f"[timbot_launch] ERROR building {stage_name}: {e}", flush=True)
        return [EmitEvent(event=ShutdownEvent(reason=f"Failed to build {stage_name}"))]

    current_actions.insert(0, LogInfo(msg=f'[timbot_launch] LAUNCHING: {stage_name}'))

    next_actions = build_stage_chain(stages_info, current_index + 1, config, sim, context)

    if not next_actions:
        return current_actions

    expected_topics = stage_cfg.get('expected_topics', [])
    delay_sec = float(stage_cfg.get('delay_sec', 2.0))

    if use_topic_check and expected_topics:
        waiter_proc = _make_topic_waiter(
            stage_name, expected_topics, config_key
        )
        event_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=waiter_proc,
                on_exit=next_actions,
            )
        )
        current_actions.append(waiter_proc)
        current_actions.append(event_handler)
    else:
        # Fallback to standard timer delay if use_topic_check is False, or expected_topics is empty
        timer = TimerAction(
            period=delay_sec,
            actions=next_actions
        )
        current_actions.append(timer)

    return current_actions


def orchestrate_launch(context: LaunchContext) -> list:
    config_file = context.launch_configurations.get('config', 'sim.yaml')
    pkg_share = get_package_share_directory('timbot_launch')
    config_path = os.path.join(pkg_share, 'config', config_file)

    if not os.path.exists(config_path):
        print(f"\n[timbot_launch] ERROR: Config file not found: {config_path}", flush=True)
        return [EmitEvent(event=ShutdownEvent(reason=f"Config file not found: {config_path}"))]

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    sim = config.get('sim', True)
    use_topic_check = config.get('use_topic_check', True)
    team_laptop = config.get('team_laptop', False)

    # Ensure ROS Python nodes launched via /usr/bin/env python3 resolve against
    # the system interpreter expected by ROS Humble, not an active Conda env.
    python_path = os.environ.get('PATH', '')
    force_system_python = [
        SetEnvironmentVariable(
            name='PATH',
            value=f"/usr/bin:{python_path}",
        )
    ]

    laptop_env_vars = [
        SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb'),
        SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1'),
        SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia')
    ] if team_laptop else []


    print(f"\n[timbot_launch] Using config: {config_file}", flush=True)
    print(f"[timbot_launch] Simulation mode: {sim}", flush=True)
    print(f"[timbot_launch] Topic Checking: {'Enabled' if use_topic_check else 'Disabled (Timer Mode)'}", flush=True)
    if not sim:
        print(f"[timbot_launch] Hardware drivers will be started sequentially before main pipeline.", flush=True)
    print(f"[timbot_launch] {'='*50}\n", flush=True)

    active_stages = [stage for stage in LAUNCH_STAGES if config.get(stage[1], {}).get('enabled', False)]

    if not active_stages and sim:
        print("[timbot_launch] ERROR: No stages enabled in config.", flush=True)
        return []

    # Build the main pipeline chain first (it becomes the "tail" for drivers).
    pipeline_actions = build_stage_chain(active_stages, 0, config, sim, context)

    if sim:
        # Simulation mode: skip hardware drivers entirely.
        return force_system_python + laptop_env_vars + pipeline_actions

    # Real rover mode (sim=False): launch hardware drivers sequentially, then
    # hand off to the normal pipeline once all drivers are confirmed ready.
    print("[timbot_launch] Starting hardware driver sequence...", flush=True)
    hardware_driver_stages = build_hardware_driver_stages(config)
    
    driver_chain = build_driver_chain(
        hardware_driver_stages, 0, use_topic_check, pipeline_actions
    )
    
    # Prepend the env vars to the driver chain
    return force_system_python + laptop_env_vars + driver_chain


# =============================================================================
# Launch Description
# =============================================================================

def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='sim.yaml',
        description='Name of the YAML config file in timbot_launch/config/'
    )

    return LaunchDescription([
        config_arg,
        OpaqueFunction(function=orchestrate_launch),
    ])