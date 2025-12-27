from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Path to the navsat config file
    navsat_config = PathJoinSubstitution([
        FindPackageShare('odom'),
        'config',
        'navsat.yaml'
    ])
    
    # NavSat transform node
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        parameters=[navsat_config],
        remappings=[
            ('odometry/filtered', 'odometry/global')
        ],
        respawn=True,
        output='screen'
    )

    return LaunchDescription([
        navsat_transform_node
    ])