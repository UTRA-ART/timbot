"""
Timbot Central Launch Orchestrator
===================================
Launches all modules sequentially based on a YAML config file.
Stages are spaced out using TimerAction delays so each stage has
time to start before the next one begins.

Usage:
  ros2 launch timbot_launch timbot.launch.py config:=sim.yaml
"""

import os
import yaml

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
    LogInfo,
    EmitEvent,
    TimerAction,
)
from launch.events import Shutdown as ShutdownEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


# =============================================================================
# Stage Launchers
# =============================================================================

def launch_gazebo(config: dict, sim: bool, context: LaunchContext) -> list:
    """Launch Gazebo simulator."""
    gazebo_cfg = config.get('gazebo', {})
    if not gazebo_cfg.get('enabled', False):
        return []

    gui = str(gazebo_cfg.get('gui', True)).lower()
    world_file = str(gazebo_cfg.get('world_file', 'track.world'))

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_worlds'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'gui': gui,
            'world_file': world_file,
        }.items()
    )
    return [gazebo_launch]


def launch_spawn(config: dict, sim: bool, context: LaunchContext) -> list:
    """Launch robot spawn (RSP, bridge, entity spawn)."""
    spawn_cfg = config.get('spawn', {})
    if not spawn_cfg.get('enabled', False):
        return []

    mute = spawn_cfg.get('mute_warnings', False)

    launch_args = {
        'sim': str(sim).lower(),
        'mute_warnings': str(mute).lower(),
        'x': str(spawn_cfg.get('x', '-19.5')),
        'y': str(spawn_cfg.get('y', '0')),
        'z': str(spawn_cfg.get('z', '0.05')),
        'roll': str(spawn_cfg.get('roll', '0')),
        'pitch': str(spawn_cfg.get('pitch', '0')),
        'yaw': str(spawn_cfg.get('yaw', '1.5708')),
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
    """Launch odom_state (EKF local + global + navsat)."""
    odom_cfg = config.get('odom_state', {})
    if not odom_cfg.get('enabled', False):
        return []

    mute = odom_cfg.get('mute_warnings', False)

    launch_args = {
        'sim': str(sim).lower(),
        'mute_warnings': str(mute).lower(),
    }

    # Pass config_file if specified
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


def launch_cartographer(config: dict, sim: bool, context: LaunchContext) -> list:
    """Launch Cartographer SLAM nodes."""
    carto_cfg = config.get('cartographer', {})
    if not carto_cfg.get('enabled', False):
        return []

    use_sim_time = str(sim).lower()
    mute = carto_cfg.get('mute_warnings', False)
    log_level = 'error' if mute else 'warn'

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
            '--ros-args', '--log-level', log_level
        ],
        remappings=[
            ('scan', '/scan_modified'),
            ('imu', '/imu/data'),
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
            '--ros-args', '--log-level', log_level
        ]
    )

    return [cartographer_node, occupancy_grid_node]


def launch_filter_lidar(config: dict, sim: bool, context: LaunchContext) -> list:
    """Launch filter_lidar_data node."""
    lidar_cfg = config.get('filter_lidar', {})
    if not lidar_cfg.get('enabled', False):
        return []

    mute = lidar_cfg.get('mute_warnings', False)

    filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('filter_lidar_data'),
            '/launch/filter_lidar_data.launch.py'
        ]),
        launch_arguments={
            'sim': str(sim).lower(),
            'mute_warnings': str(mute).lower(),
        }.items()
    )
    return [filter_launch]


def launch_rviz(config: dict, sim: bool, context: LaunchContext) -> list:
    """Launch RViz visualization."""
    rviz_cfg = config.get('rviz', {})
    if not rviz_cfg.get('enabled', False):
        return []

    rviz_config = PathJoinSubstitution([
        FindPackageShare("description"),
        "rviz",
        "timbot.rviz"
    ])

    mute = rviz_cfg.get('mute_warnings', False)
    log_level = 'error' if mute else 'warn'

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config, "--ros-args", "--log-level", log_level],
        parameters=[{'use_sim_time': str(sim).lower()}],
        output="screen"
    )
    return [rviz_node]


def launch_nav_stack(config: dict, sim: bool, context: LaunchContext) -> list:
    """Launch Nav2 stack."""
    nav_cfg = config.get('nav_stack', {})
    if not nav_cfg.get('enabled', False):
        return []

    mute = nav_cfg.get('mute_warnings', False)

    launch_args = {
        'sim': str(sim).lower(),
        'mute_warnings': str(mute).lower(),
    }

    # Pass config_file if specified
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
    """Launch waypoint navigation."""
    wp_cfg = config.get('load_waypoints', {})
    if not wp_cfg.get('enabled', False):
        return []

    mute = wp_cfg.get('mute_warnings', False)

    launch_args = {
        'sim': str(sim).lower(),
        'mute_warnings': str(mute).lower(),
    }

    # Pass config_file (waypoints json) if specified
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
# Sequential Orchestrator
# =============================================================================

# Define launch stages in order.
# Each entry: (stage_name, config_key, launcher_function, delay_seconds)
# delay_seconds = how long to wait AFTER this stage launches before starting the next
LAUNCH_STAGES = [
    ('Gazebo',           'gazebo',         launch_gazebo,         10.0),
    ('Spawn',            'spawn',          launch_spawn,          10.0),
    ('Odom State',       'odom_state',     launch_odom_state,      5.0),
    ('Cartographer',     'cartographer',   launch_cartographer,    5.0),
    ('Filter Lidar',     'filter_lidar',   launch_filter_lidar,    3.0),
    ('RViz',             'rviz',           launch_rviz,            2.0),
    ('Nav Stack',        'nav_stack',      launch_nav_stack,      10.0),
    ('Load Waypoints',   'load_waypoints', launch_load_waypoints,  0.0),
]


def orchestrate_launch(context: LaunchContext) -> list:
    """
    Main orchestration function. Reads the config YAML and launches
    each stage sequentially using TimerActions with cumulative delays.
    """
    # --- Load config ---
    config_file = context.launch_configurations.get('config', 'sim.yaml')
    pkg_share = get_package_share_directory('timbot_launch')
    config_path = os.path.join(pkg_share, 'config', config_file)

    if not os.path.exists(config_path):
        print(f"\n[timbot_launch] ERROR: Config file not found: {config_path}")
        return [EmitEvent(event=ShutdownEvent(reason=f"Config file not found: {config_path}"))]

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    sim = config.get('sim', True)
    print(f"\n[timbot_launch] Using config: {config_file}")
    print(f"[timbot_launch] Simulation mode: {sim}")
    print(f"[timbot_launch] {'='*50}\n")

    all_actions = []
    cumulative_delay = 0.0

    for stage_name, config_key, launcher_fn, stage_delay in LAUNCH_STAGES:
        stage_cfg = config.get(config_key, {})

        # Skip disabled stages
        if not stage_cfg.get('enabled', False):
            print(f"[timbot_launch] SKIP: {stage_name} (disabled)")
            continue

        # Build the stage actions
        try:
            actions = launcher_fn(config, sim, context)
        except Exception as e:
            print(f"[timbot_launch] ERROR building {stage_name}: {e}")
            return [EmitEvent(event=ShutdownEvent(reason=f"Failed to build {stage_name}: {e}"))]

        if not actions:
            continue

        # Schedule with TimerAction; first enabled stage launches immediately
        log_action = LogInfo(msg=f'[timbot_launch] LAUNCHING: {stage_name}...')
        if cumulative_delay > 0:
            all_actions.append(
                TimerAction(
                    period=cumulative_delay,
                    actions=[log_action, *actions]
                )
            )
        else:
            all_actions.append(log_action)
            all_actions.extend(actions)

        cumulative_delay += stage_delay

    print(f"[timbot_launch] All stages scheduled (total delay: {cumulative_delay}s)")

    return all_actions


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
