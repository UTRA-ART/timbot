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
    width_arg = DeclareLaunchArgument('camera_width', default_value='640')
    height_arg = DeclareLaunchArgument('camera_height', default_value='320')
    white_sensitivity_arg = DeclareLaunchArgument('white_sensitivity', default_value='20')
    downscale_factor_arg = DeclareLaunchArgument('downscale_factor', default_value='1')
    horizon_crop_arg = DeclareLaunchArgument('horizon_crop', default_value='0.15')
    morph_size_arg = DeclareLaunchArgument('morph_size', default_value='3')
    morph_open_iters_arg = DeclareLaunchArgument('morph_open_iters', default_value='1')
    morph_close_iters_arg = DeclareLaunchArgument('morph_close_iters', default_value='1')

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
            'camera_width': LaunchConfiguration('camera_width'),
            'camera_height': LaunchConfiguration('camera_height'),
            'white_sensitivity': LaunchConfiguration('white_sensitivity'),
            'downscale_factor': LaunchConfiguration('downscale_factor'),
            'horizon_crop': LaunchConfiguration('horizon_crop'),
            'morph_size': LaunchConfiguration('morph_size'),
            'morph_open_iters': LaunchConfiguration('morph_open_iters'),
            'morph_close_iters': LaunchConfiguration('morph_close_iters'),
        }],
        remappings=[('image', '/zed_node/left/image')],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        sim_arg,
        log_level_arg,
        lane_detection_mode_arg,
        width_arg,
        height_arg,
        white_sensitivity_arg,
        downscale_factor_arg,
        horizon_crop_arg,
        morph_size_arg,
        morph_open_iters_arg,
        morph_close_iters_arg,
        lane_detection_inference,
    ])
