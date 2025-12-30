from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Declare launch arguments
    launch_state_arg = DeclareLaunchArgument(
        'launch_state',
        default_value='sim',
        description='Launch state parameter'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame for the robot'
    )
    
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame'
    )
    
    # Configuration directory path
    config_dir = PathJoinSubstitution([
        FindPackageShare('description'),
        'config'
    ])
    
    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'cartographer.lua'
        ],
        remappings=[
            ('imu', '/imu/data'),
            ('odom', '/odom'),
            ('scan', '/scan_modified'),
            ('scan_1', '/cv/lane_detections_scan'),
            ('scan_2', '/scan_modified'),
            ('fix', '/gps/fix')
        ]
    )
    
    # Cartographer occupancy grid node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=['-resolution', '0.05']
    )

    return LaunchDescription([
        launch_state_arg,
        base_frame_arg,
        odom_frame_arg,
        cartographer_node,
        occupancy_grid_node
    ])