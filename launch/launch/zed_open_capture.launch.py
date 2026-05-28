from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video2',
        description='Video device path',
    )

    auto_exposure_arg = DeclareLaunchArgument(
        'auto_exposure',
        default_value='true',
        description='Enable automatic exposure/gain control',
    )

    exposure_arg = DeclareLaunchArgument(
        'exposure',
        default_value='50',
        description='Manual exposure value in the range 0-100',
    )

    gain_arg = DeclareLaunchArgument(
        'gain',
        default_value='50',
        description='Manual gain value in the range 0-100',
    )

    gamma_arg = DeclareLaunchArgument(
        'gamma',
        default_value='5',
        description='Manual gamma value in the range 1-9',
    )

    num_disparities_arg = DeclareLaunchArgument(
        'num_disparities',
        default_value='96',
        description='StereoSGBM disparity count (multiple of 16)',
    )

    block_size_arg = DeclareLaunchArgument(
        'block_size',
        default_value='3',
        description='StereoSGBM block size (odd integer)',
    )

    p1_multiplier_arg = DeclareLaunchArgument(
        'p1_multiplier',
        default_value='8',
        description='Multiplier used to compute StereoSGBM P1',
    )

    p2_multiplier_arg = DeclareLaunchArgument(
        'p2_multiplier',
        default_value='32',
        description='Multiplier used to compute StereoSGBM P2',
    )

    left_image_topic_arg = DeclareLaunchArgument(
        'left_image_topic',
        default_value='/zed_node/left/image',
    )
    left_camera_info_topic_arg = DeclareLaunchArgument(
        'left_camera_info_topic',
        default_value='/zed_node/left/camera_info',
    )
    depth_image_topic_arg = DeclareLaunchArgument(
        'depth_image_topic',
        default_value='/zed_node/left/depth_image',
    )
    point_cloud_topic_arg = DeclareLaunchArgument(
        'point_cloud_topic',
        default_value='/zed_node/left/points',
    )

    zed_open_capture_node = Node(
        package='timbot_launch',
        executable='zed_open_capture_node',
        name='zed_open_capture',
        output='screen',
        parameters=[
            {
                'video_device': LaunchConfiguration('video_device'),
                'auto_exposure': LaunchConfiguration('auto_exposure'),
                'exposure': LaunchConfiguration('exposure'),
                'gain': LaunchConfiguration('gain'),
                'gamma': LaunchConfiguration('gamma'),
                'num_disparities': LaunchConfiguration('num_disparities'),
                'block_size': LaunchConfiguration('block_size'),
                'p1_multiplier': LaunchConfiguration('p1_multiplier'),
                'p2_multiplier': LaunchConfiguration('p2_multiplier'),
                'left_image_topic': LaunchConfiguration('left_image_topic'),
                'left_camera_info_topic': LaunchConfiguration('left_camera_info_topic'),
                'depth_image_topic': LaunchConfiguration('depth_image_topic'),
                'point_cloud_topic': LaunchConfiguration('point_cloud_topic'),
            }
        ],
    )

    return LaunchDescription([
        video_device_arg,
        auto_exposure_arg,
        exposure_arg,
        gain_arg,
        gamma_arg,
        num_disparities_arg,
        block_size_arg,
        p1_multiplier_arg,
        p2_multiplier_arg,
        left_image_topic_arg,
        left_camera_info_topic_arg,
        depth_image_topic_arg,
        point_cloud_topic_arg,
        zed_open_capture_node,
    ])
