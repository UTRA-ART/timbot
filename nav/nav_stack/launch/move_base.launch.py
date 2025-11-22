from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch arguments
    declare_sim_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Get package directory
    nav_stack_dir = get_package_share_directory('nav_stack')
    config_dir = os.path.join(nav_stack_dir, 'config')

    # YAML config files
    costmap_common = os.path.join(config_dir, 'costmap_common_params.yaml')
    local_costmap = os.path.join(config_dir, 'local_costmap_params.yaml')
    global_costmap = os.path.join(config_dir, 'global_costmap_params.yaml')
    planners = os.path.join(config_dir, 'local_global_planner.yaml')

    # Nav2 parameter file
    nav2_params = os.path.join(config_dir, 'nav2_params.yaml')

    # Nav2 brings multiple nodes (planner, controller, costmaps, bt_navigator, etc.)
    # Normally, they are launched together using nav2_bringup
    # but here's a custom standalone setup

    # Controller (local planner)
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[costmap_common, local_costmap, planners, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('cmd_vel', 'nav_vel')]
    )

    # Planner (global planner)
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[costmap_common, global_costmap, planners, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Behavior Tree Navigator (replaces move_base)
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Lifecycle manager (brings all Nav2 nodes up)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'bt_navigator'
            ]
        }]
    )

    return LaunchDescription([
        declare_sim_arg,
        controller_node,
        planner_node,
        bt_navigator_node,
        lifecycle_manager_node
    ])
