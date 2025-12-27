# sample odom launch file

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Declare launch arguments (to match expected interface)
    launch_state_arg = DeclareLaunchArgument(
        'launch_state',
        default_value='sim',
        description='Launch state (sim or real) - mock implementation'
    )
    
    # Mock odometry - just publishes odom->base_link transform
    mock_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='mock_odom_tf',
        arguments=[
            '0', '0', '0',  # x, y, z
            '0', '0', '0', '1',  # qx, qy, qz, qw (identity quaternion)
            'odom',  # parent frame
            'base_link'  # child frame
        ],
        output='screen'
    )
    
    # Optional: Mock odometry message publisher
    # (You can add this later if needed)

    return LaunchDescription([
        launch_state_arg,
        mock_odom_tf
    ])