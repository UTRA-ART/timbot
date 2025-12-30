from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Declare launch arguments
    cmd_vel_out_arg = DeclareLaunchArgument(
        'cmd_vel_out',
        default_value='twist_mux/cmd_vel',
        description='Output topic for cmd_vel'
    )
    
    config_locks_arg = DeclareLaunchArgument(
        'config_locks',
        default_value=PathJoinSubstitution([
            FindPackageShare('twist_mux'),
            'config',
            'twist_mux_locks.yaml'
        ]),
        description='Path to locks configuration file'
    )
    
    config_topics_arg = DeclareLaunchArgument(
        'config_topics', 
        default_value=PathJoinSubstitution([
            FindPackageShare('twist_mux'),
            'config', 
            'twist_mux_topics.yaml'
        ]),
        description='Path to topics configuration file'
    )
    
    # Twist mux node
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            LaunchConfiguration('config_locks'),
            LaunchConfiguration('config_topics')
        ],
        remappings=[
            ('cmd_vel_out', LaunchConfiguration('cmd_vel_out'))
        ]
    )
    
    # Twist marker node
    twist_marker_node = Node(
        package='twist_mux',
        executable='twist_marker', 
        name='twist_marker',
        remappings=[
            ('twist', LaunchConfiguration('cmd_vel_out')),
            ('marker', 'twist_marker')
        ]
    )

    return LaunchDescription([
        cmd_vel_out_arg,
        config_locks_arg,
        config_topics_arg,
        twist_mux_node,
        twist_marker_node
    ])