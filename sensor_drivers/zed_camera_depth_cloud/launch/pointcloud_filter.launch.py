from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration('input_pointcloud_topic')
    filtered_topic = LaunchConfiguration('pointcloud_topic')
    obstacle_topic = LaunchConfiguration('obstacle_pointcloud_topic')
    ramp_topic = LaunchConfiguration('ramp_pointcloud_topic')
    frame_id = LaunchConfiguration('frame_id')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('input_pointcloud_topic', default_value='/zed_node/left/points'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/zed_node/left/points/filtered'),
        DeclareLaunchArgument(
            'obstacle_pointcloud_topic',
            default_value='/zed_node/left/points/obstacles',
        ),
        DeclareLaunchArgument('ramp_pointcloud_topic', default_value='/zed_node/left/points/ramps'),
        DeclareLaunchArgument('frame_id', default_value='left_camera_link_optical'),
        Node(
            package='zed_camera_depth_cloud',
            executable='pointcloud_filter.py',
            name='pointcloud_filter',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_pointcloud_topic': input_topic,
                'pointcloud_topic': filtered_topic,
                'obstacle_pointcloud_topic': obstacle_topic,
                'ramp_pointcloud_topic': ramp_topic,
                'frame_id': frame_id,
            }],
        ),
    ])
