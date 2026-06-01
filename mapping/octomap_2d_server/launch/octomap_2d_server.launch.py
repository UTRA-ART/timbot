from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='octomap_params.yaml',
        description='OctoMap server parameter file',
    )
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan_modified',
        description='LaserScan topic to convert',
    )
    depth_cloud_topic_arg = DeclareLaunchArgument(
        'depth_cloud_topic',
        default_value='/zed_node/left/obstacle_points',
        description='Depth camera PointCloud2 topic',
    )
    combined_cloud_topic_arg = DeclareLaunchArgument(
        'combined_cloud_topic',
        default_value='/combined_cloud',
        description='Merged PointCloud2 output topic',
    )
    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Static transform parent frame',
    )
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Static transform child frame',
    )
    publish_static_tf_arg = DeclareLaunchArgument(
        'publish_static_tf',
        default_value='true',
        description='Publish a static map->odom identity transform',
    )

    params_file = PathJoinSubstitution([
        FindPackageShare('octomap_2d_server'),
        'config',
        LaunchConfiguration('config_file'),
    ])

    cloud_merge = Node(
        package='octomap_2d_server',
        executable='pointcloud_merge_node.py',
        name='octomap_cloud_merge',
        output='screen',
        parameters=[{
            'scan_topic': LaunchConfiguration('scan_topic'),
            'depth_cloud_topic': LaunchConfiguration('depth_cloud_topic'),
            'output_topic': LaunchConfiguration('combined_cloud_topic'),
        }],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            LaunchConfiguration('map_frame'),
            LaunchConfiguration('odom_frame'),
        ],
        condition=IfCondition(LaunchConfiguration('publish_static_tf')),
        output='screen',
    )

    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('projected_map', 'octomap_2d'),
            ('cloud_in', LaunchConfiguration('combined_cloud_topic')),
        ],
    )

    return LaunchDescription([
        config_file_arg,
        scan_topic_arg,
        depth_cloud_topic_arg,
        combined_cloud_topic_arg,
        map_frame_arg,
        odom_frame_arg,
        publish_static_tf_arg,
        cloud_merge,
        static_tf,
        octomap_server,
    ])