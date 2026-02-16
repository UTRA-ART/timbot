from launch import LaunchDescription

#used to define launch arguments that can be passed from the launch file or the console
from launch.actions import DeclareLaunchArgument

#gives value of the launch argument in any part of launch description
from launch.substitutions import LaunchConfiguration

#node class to launch ROS2 nodes
from launch_ros.actions import Node

#returns the share directory of the given package 
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    #creates launch argument named use_sim_time
    #default value is true, meaning nodes will use simulation time from Gazebo
    use_sim_time_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('sim')

    # Get package directory
    pkg_dir = get_package_share_directory('nav_stack')
    config_dir = os.path.join(pkg_dir, 'config')

    # Nav2 parameter file (contains costmap params, planner, controller, bt_navigator)
    nav2_params = os.path.join(config_dir, 'nav2_params.yaml')

    # In Nav2, costmaps are integrated into planner_server and controller_server
    # There is no standalone 'costmap_server' executable
    
    # Behavior Tree XML file for navigation logic
    bt_xml = os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    # Planner Server (global planner + global costmap)
    # Computes global paths from robot pose to goal pose
    # The global_costmap parameters are loaded from nav2_params.yaml
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    # Controller Server (local planner + local costmap)
    # Computes velocity commands (cmd_vel) to follow global path
    # The local_costmap parameters are loaded from nav2_params.yaml
    controller_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],\
        arguments=['--ros-args', '--log-level', 'warn'],
        remappings=[("cmd_vel", "nav_vel")]
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # Behavior Tree Navigator (replaces move_base)
    #runs behaviour tree xml file to coordinate navigation
    #navigation logic
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time, "default_bt_xml_filename": bt_xml}],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # Lifecycle manager (brings all Nav2 nodes up)
    # Starts all nodes in correct order
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'planner_server',
                'controller_server',
                'bt_navigator',
                'behavior_server'
            ]
        }],
        arguments=['--ros-args', '--log-level', 'warn']
    )


    # LaunchDescription is a container for actions, collects nodes, launch arguments, etc.
    # Executes all actions in order
    return LaunchDescription([
        use_sim_time_arg,      # Declares launch argument for simulation time
        planner_node,         # Global planner (includes global costmap)
        controller_node,      # Local controller/planner (includes local costmap)
        behavior_server_node, # Behavior server for recovery behaviors
        bt_navigator_node,    # Behavior tree navigator
        lifecycle_manager_node  # Lifecycle manager
    ])
