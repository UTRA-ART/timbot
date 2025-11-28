from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    launch_state = LaunchConfiguration('launch_state')

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_state',
            default_value='sim'
        ),

        Node(
            package='load_waypoints',
            executable='nav_options.py',
            name='nav_control',
            output='screen'
        )
    ])
