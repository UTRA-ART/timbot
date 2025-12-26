from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Get absolute path to igvc_ramp.world file
    ramp_file = os.path.join(os.getcwd(), 'igvc_ramp.world')
    
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', ramp_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo
    ])