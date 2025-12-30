from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Declare launch argument
    visual_odom_arg = DeclareLaunchArgument(
        'visual_odom',
        default_value='false',
        description='Enable visual odometry'
    )
    
    # Scheduler node
    scheduler_node = Node(
        package='description',
        executable='scheduler.py',
        name='scheduler',
        output='screen',
        parameters=[
            {'visual_odom_enable': LaunchConfiguration('visual_odom')}
        ]
    )

    return LaunchDescription([
        visual_odom_arg,
        scheduler_node
    ])