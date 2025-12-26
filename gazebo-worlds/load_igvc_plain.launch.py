from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Get absolute path to igvc_plain.world file
    plain_file = os.path.join(os.getcwd(), 'igvc_plain.world')
    
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', plain_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo
    ])