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

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
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
from launch_ros.actions import Node
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


def launch_spawn(config: dict, sim: bool, context: LaunchContext) -> list:
    spawn_cfg = config.get('spawn', {})
    log_level = spawn_cfg.get('log_level', 'info')

    launch_args = {
        'sim': str(sim).lower(),
        'log_level': log_level,
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
    odom_cfg = config.get('odom_state', {})
    log_level = odom_cfg.get('log_level', 'info')
    launch_args = {
        'sim': str(sim).lower(),
        'log_level': log_level,
        'position_covariance': str(odom_cfg.get('position_covariance', 0.05)),
        'orientation_covariance': str(odom_cfg.get('orientation_covariance', 0.01)),
        'horizontal_stddev': str(odom_cfg.get('horizontal_stddev', 3.0)),
        'vertical_stddev': str(odom_cfg.get('vertical_stddev', 4.0)),
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
    }
    filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('filter_lidar_data'),
            '/launch/filter_lidar_data.launch.py'
        ]),
        launch_arguments=filter_args.items()
    )
    return [filter_launch]

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
            ('imu', '/imu/data'),
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
# Sequential Event-Driven Orchestrator
# =============================================================================

LAUNCH_STAGES = [
    ('Gazebo',         'gazebo',         launch_gazebo),
    ('Spawn',          'spawn',          launch_spawn),
    ('Odom State',     'odom_state',     launch_odom_state),
    ('Filter Lidar',   'filter_lidar',   launch_filter_lidar),
    ('Cartographer',   'cartographer',   launch_cartographer),
    ('RViz',           'rviz',           launch_rviz),
    ('Nav Stack',      'nav_stack',      launch_nav_stack),
    ('Load Waypoints', 'load_waypoints', launch_load_waypoints),
]

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
        topics_str = " ".join(expected_topics)
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

        waiter_proc = ExecuteProcess(
            cmd=['sh', '-c', wait_cmd],
            name=f'waiter_{config_key}',
            output='screen'
        )

        event_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=waiter_proc,
                on_exit=next_actions
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
    
    print(f"\n[timbot_launch] Using config: {config_file}", flush=True)
    print(f"[timbot_launch] Simulation mode: {sim}", flush=True)
    print(f"[timbot_launch] Topic Checking: {'Enabled' if use_topic_check else 'Disabled (Timer Mode)'}", flush=True)
    print(f"[timbot_launch] {'='*50}\n", flush=True)

    active_stages = [stage for stage in LAUNCH_STAGES if config.get(stage[1], {}).get('enabled', False)]
    
    if not active_stages:
        print("[timbot_launch] ERROR: No stages enabled in config.", flush=True)
        return []

    return build_stage_chain(active_stages, 0, config, sim, context)


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