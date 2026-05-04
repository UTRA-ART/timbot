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
        left_image_topic_arg,
        left_camera_info_topic_arg,
        depth_image_topic_arg,
        point_cloud_topic_arg,
        zed_open_capture_node,
    ])
