from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error',
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='nav.yaml',
        description='Name of the nav file in nav_stack/config/',
    )

    nav2_params_configured = PathJoinSubstitution([
            FindPackageShare('nav_stack'), 'config', LaunchConfiguration('config_file')
        ])

    planner_node = Node(
            package='nav_stack',
            executable='planner.py',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_configured, {'use_sim_time': LaunchConfiguration('sim')}],
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        )

    controller_node = Node(
                package='nav_stack',
                executable='controller.py',
                name='controller_server',
                output='screen',
                parameters=[nav2_params_configured, {'use_sim_time': LaunchConfiguration('sim')}],
                        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            )

    return LaunchDescription([
        sim_arg,
        log_level_arg,
        config_file_arg,
        planner_node,
        controller_node,
    ])
