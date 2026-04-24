import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_dir = get_package_share_directory('zed_custom')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='zed_params.yaml',
        description='Name of the ZED params YAML file'
    )
    
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video2',
        description='Video device path'
    )
    
    left_image_topic_arg = DeclareLaunchArgument('left_image_topic', default_value='/left/image_raw')
    left_camera_info_topic_arg = DeclareLaunchArgument('left_camera_info_topic', default_value='/left/camera_info')
    right_image_topic_arg = DeclareLaunchArgument('right_image_topic', default_value='/right/image_raw')
    right_camera_info_topic_arg = DeclareLaunchArgument('right_camera_info_topic', default_value='/right/camera_info')
    depth_image_topic_arg = DeclareLaunchArgument('depth_image_topic', default_value='/depth/depth_image')
    point_cloud_topic_arg = DeclareLaunchArgument('point_cloud_topic', default_value='/points2')
    
    params_file = PathJoinSubstitution([pkg_dir, 'config', LaunchConfiguration('config_file')])

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[
            params_file,
            {'video_device': LaunchConfiguration('video_device')}
        ],
        remappings=[
            ('image_raw/compressed', '_image_raw/compressed'),
            ('image_raw/compressedDepth', '_image_raw/compressedDepth'),
            ('image_raw/theora', '_image_raw/theora')
        ]
    )

    left_splitter_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::CropDecimateNode',
        name='left_splitter',
        remappings=[
            ('in/image_raw', '/image_raw'),
            ('in/camera_info', '/camera_info'),
            ('out/image_raw', LaunchConfiguration('left_image_topic')),
            ('out/camera_info', '/_left/camera_info_bad'),
            ('out/image_raw/compressed', '/_left/image_raw/compressed'),
            ('out/image_raw/compressedDepth', '/_left/image_raw/compressedDepth'),
            ('out/image_raw/theora', '/_left/image_raw/theora')
        ],
        parameters=[{
            'offset_x': 0,
            'offset_y': 0,
            'width': 672,
            'height': 376
        }]
    )
    
    right_splitter_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::CropDecimateNode',
        name='right_splitter',
        remappings=[
            ('in/image_raw', '/image_raw'),
            ('in/camera_info', '/camera_info'),
            ('out/image_raw', LaunchConfiguration('right_image_topic')),
            ('out/camera_info', '/_right/camera_info_bad'),
            ('out/image_raw/compressed', '/_right/image_raw/compressed'),
            ('out/image_raw/compressedDepth', '/_right/image_raw/compressedDepth'),
            ('out/image_raw/theora', '/_right/image_raw/theora')
        ],
        parameters=[{
            'offset_x': 672,  
            'offset_y': 0,
            'width': 672,
            'height': 376
        }]
    )

    container = ComposableNodeContainer(
        name='zed_sbs_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            left_splitter_node,
            right_splitter_node,
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::DisparityNode',
                name='disparity',
                parameters=[{'approximate_sync': True}],
                remappings=[
                    ('left/image_rect', LaunchConfiguration('left_image_topic')),
                    ('right/image_rect', LaunchConfiguration('right_image_topic')),
                    ('left/camera_info', LaunchConfiguration('left_camera_info_topic')),
                    ('right/camera_info', LaunchConfiguration('right_camera_info_topic'))
                ]
            ),
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::PointCloudNode',
                name='point_cloud',
                parameters=[{'approximate_sync': True}],
                remappings=[
                    ('left/image_rect_color', '/left/image_color'),
                    ('left/camera_info', LaunchConfiguration('left_camera_info_topic')),
                    ('right/camera_info', LaunchConfiguration('right_camera_info_topic')),
                    ('points2', LaunchConfiguration('point_cloud_topic'))
                ]
            )
        ],
    )
    
    camera_info_pub_node = Node(
        package='zed_custom',
        executable='zed_camera_info_publisher.py',
        name='zed_camera_info_publisher',
        output='screen',
        remappings=[
            ('/left/image_raw', LaunchConfiguration('left_image_topic')),
            ('/right/image_raw', LaunchConfiguration('right_image_topic')),
            ('/left/camera_info', LaunchConfiguration('left_camera_info_topic')),
            ('/right/camera_info', LaunchConfiguration('right_camera_info_topic'))
        ]
    )
    
    disparity_to_depth_node = Node(
        package='zed_custom',
        executable='disparity_to_depth.py',
        name='disparity_to_depth',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/depth/depth_image', LaunchConfiguration('depth_image_topic')),
            ('/left/image_raw', LaunchConfiguration('left_image_topic'))
        ]
    )

    return LaunchDescription([
        config_file_arg,
        video_device_arg,
        left_image_topic_arg,
        left_camera_info_topic_arg,
        right_image_topic_arg,
        right_camera_info_topic_arg,
        depth_image_topic_arg,
        point_cloud_topic_arg,
        usb_cam_node,
        container,
        camera_info_pub_node,
        disparity_to_depth_node
    ])