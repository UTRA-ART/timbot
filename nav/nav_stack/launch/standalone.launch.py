"""
Standalone nav_stack launch for testing without Gazebo.

Usage:
  Terminal 1: ros2 run nav_stack fake_robot_sim.py
  Terminal 2: ros2 launch nav_stack standalone.launch.py
  Terminal 3: ros2 run nav_stack send_nav_goal.py --x 2 --y 1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('nav_stack')
    config_dir = os.path.join(pkg_dir, 'config')
    
    # Standalone params file
    nav2_params = os.path.join(config_dir, 'nav2_standalone_params.yaml')
    bt_xml = os.path.join(config_dir, 'navigate_simple.xml')
    
    # Planner Server
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    # Controller Server
    controller_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params],
        remappings=[("cmd_vel", "nav_vel")]
    )

    # Behavior Tree Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params, 
            {
                'default_nav_to_pose_bt_xml': bt_xml,
                'default_nav_through_poses_bt_xml': bt_xml
            }
        ],
    )

    # Lifecycle manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'planner_server',
                'controller_server',
                'bt_navigator'
            ]
        }]
    )

    return LaunchDescription([
        planner_node,
        controller_node,
        bt_navigator_node,
        lifecycle_manager_node,
    ])
