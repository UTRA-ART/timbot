from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Get absolute path to world file
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    world_file = os.path.join(launch_dir, 'igvc_full.world')
    
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo
    ])
