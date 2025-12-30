from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    
    # Declare launch arguments
    camera_ns_arg = DeclareLaunchArgument(
        'camera_ns',
        default_value='zed_node',
        description='Camera namespace'
    )
    
    # Create group with namespace
    zed_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('camera_ns')),
            
            # Image Rectification + Stereo Processing
            Node(
                package='stereo_image_proc',
                executable='disparity_node',
                name='stereo_image_proc',
                remappings=[
                    ('points2', 'point_cloud/cloud_registered'),
                    ('disparity', 'disparity/disparity_registered'),
                    ('left/image_raw', 'left/image_raw_color'),
                    ('right/image_raw', 'right/image_raw_color')
                ],
                parameters=[
                    {'speckle_size': 400}
                ]
            ),
            
            # Disparity to depth
            Node(
                package='rtabmap_util', 
                executable='disparity_to_depth',
                name='disparity2depth',
                remappings=[
                    ('depth', 'depth/depth_registered'),
                    ('disparity', 'disparity/disparity_registered')
                ]
            ),
        ]
    )

    return LaunchDescription([
        camera_ns_arg,
        zed_group
    ])