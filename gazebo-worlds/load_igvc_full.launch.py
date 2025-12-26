from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Get absolute path to world file
    world_file = os.path.join(os.getcwd(), 'igvc_full.world')
    
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo
    ])
