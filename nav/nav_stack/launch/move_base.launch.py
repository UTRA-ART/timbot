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
    declare_sim_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Get package directory
    pkg_dir = get_package_share_directory('nav_stack')
    config_dir = os.path.join(pkg_dir, 'config')

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
    
    #finds path to a built-in Behaviour Tree XML file (comes with Nav2)
    #replaces move_base functionality
    #contains nodes for later

    bt_xml = os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    #nodes break navigation into smaller parts
    #global_costmap used by planner_server
    #considers where the robot can go in the entire map

    global_costmap_node  = Node(
        package='nav2_costmap_2d',
        executable='costmap_server',
        name='global_costmap',
        output='screen',
        parameters=[costmap_common, global_costmap, planners, {'use_sim_time': LaunchConfiguration('use_sim_time')}], #launch argument is now connected to the parameters, meaning they follow simulated time
    )

    #local_costmap used by controller_server
    #focuses on immediate surroundings of the robot for short-term path adjustments
    local_costmap_node  = Node(
        package='nav2_costmap_2d',
        executable='costmap_server',
        name='local_costmap',
        output='screen',
        parameters=[costmap_common, local_costmap, planners, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )


    #Planner (global planner)
    #computes global paths from robot pose to goal pose
    #uses global_costmap_node API, queries obstacle info to avoid obstacles
    #essentially reads global map under namespace global_costmap and plans path
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planners, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    #local planner (DWB controller)
    #computes velocity commands (cmd_vel) to follow global path
    #reads local_costmap to return motor commands
    controller_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[planners, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[("cmd_vel", "nav_vel")]
    )

    # Behavior Tree Navigator (replaces move_base)
    #runs behaviour tree xml file to coordinate navigation
    #navigation logic
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': LaunchConfiguration('use_sim_time'), "default_bt_xml_filename": bt_xml}],
    )

    # Lifecycle manager (brings all Nav2 nodes up)
    #starts all nodes in correct order
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
                'local_costmap',
                'global_costmap',
                'bt_navigator'
            ]
        }]
    )


    #LaunchDescription is a container for actions, collects nodes, launch arguments, etc.
    #executes all actions in order
    return LaunchDescription([
        declare_sim_arg, #declares launch argument so you can switch/set simulation time vs. real time
        global_costmap_node, #starts global costmap server
        local_costmap_node, #starts local costmap server
        planner_node, #reads global costmap and starts planner server
        controller_node, #reads local costmap and starts controller (DWB) server
        bt_navigator_node, #starts behaviour tree navigator
        lifecycle_manager_node #starts lifecycle manager
    ])
