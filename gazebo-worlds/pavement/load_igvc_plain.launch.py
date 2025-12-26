from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Get absolute path to igvc_plain.world file
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    plain_file = os.path.join(launch_dir, 'igvc_plain.world')
    
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', plain_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo
    ])