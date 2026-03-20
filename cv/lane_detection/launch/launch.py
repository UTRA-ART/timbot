# Run with: `ros2 launch lane_detection launch.py`
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    sim_arg = DeclareLaunchArgument('sim', default_value='true')
    log_level_arg = DeclareLaunchArgument('log_level', default_value='info')
    lane_detection_mode_arg = DeclareLaunchArgument(
        'lane_detection_mode', default_value='0',
        description='0 = deep learning (YOLO), 1 = classical (HSV threshold)'
    )

    use_sim_time = LaunchConfiguration('sim')
    log_level = LaunchConfiguration('log_level')
    lane_detection_mode = LaunchConfiguration('lane_detection_mode')

    lane_detection_inference = Node(
        package='lane_detection',
        executable='lane_detection_inference',
        name='lane_detection_inference',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'lane_detection_mode': lane_detection_mode,
        }],
        remappings=[('image', '/zed_node/left/image')],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        sim_arg,
        log_level_arg,
        lane_detection_mode_arg,
        lane_detection_inference,
    ])
