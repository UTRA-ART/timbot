from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Get absolute path to world file
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    world_file = os.path.join(launch_dir, 'igvc_full.world')
    
    # Set Gazebo resource path to find models in workspace
    workspace_root = os.path.dirname(os.path.dirname(launch_dir))
    models_dir = os.path.join(workspace_root, 'models')
    home_dir = os.path.expanduser('~')
    
    env = os.environ.copy()
    env['GZ_SIM_RESOURCE_PATH'] = models_dir + ':' + os.path.join(home_dir, '.gazebo/models') + ':' + env.get('GZ_SIM_RESOURCE_PATH', '')
    
    gazebo = ExecuteProcess(
        #cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file],
        cmd=['ign', 'gazebo', 'sim', world_file, '--verbose'],
        output='screen',
        env=env
    )

    return LaunchDescription([
        gazebo
    ])
