from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    launch_state = LaunchConfiguration('launch_state')

    return LaunchDescription([
        # Declare launch argument
        DeclareLaunchArgument(
            'launch_state',
            default_value='sim'
        ),

        # load_waypoints_server (Python script installed via install(PROGRAMS) in CMakeLists)
        Node(
            package='load_waypoints',
            executable='navigate_waypoints.py',
            name='load_waypoints_server',
            output='screen',
            parameters=[{
                'launch_state': launch_state
            }]
        ),

        # ramp_navigate executable
        Node(
            package='load_waypoints',
            executable='ramp_navigate',
            name='ramp_navigate',
            output='screen'
        )
    ])