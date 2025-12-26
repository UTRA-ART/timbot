from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Get absolute path to igvc_walls.world file
    walls_file = os.path.join(os.getcwd(), 'igvc_walls.world')
    
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', walls_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo
    ])