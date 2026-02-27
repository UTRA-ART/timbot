import sys
from launch import LaunchDescription
from launch.actions import ExecuteProcess, Shutdown, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import os

def generate_launch_description():
    # --- Arguments ---
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run Gazebo in headless mode (server only) if True'
    )
    gui = LaunchConfiguration('gui')

    # --- Path Setup ---
    # Get absolute path to world file
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    world_file = os.path.join(launch_dir, 'igvc_full.world')
    
    # Set Gazebo resource path to find models in workspace
    home_dir = os.path.expanduser('~')
    gazebo_models_path = os.path.join(home_dir, '.gazebo/models')

    env = os.environ.copy()
    env['IGN_GAZEBO_RESOURCE_PATH'] = gazebo_models_path
    env['LIBGL_ALWAYS_SOFTWARE'] = '1'
    
    # --- Gazebo Execution Logic ---
    
    # Option 1: Full GUI (Run if headless is False)
    gazebo_gui = ExecuteProcess(
        # cmd=['ign', 'gazebo', 'sim', '-r', world_file, '--verbose'],
        cmd=['ign', 'gazebo', 'sim', '-r', world_file],
        output='screen',
        env=env,
        on_exit=Shutdown(),
        condition=IfCondition(gui) # Only run if gui is TRUE
    )

    # Option 2: Headless / Server Only (Run if gui is True)
    # We add the '-s' flag to run only the server
    gazebo_headless = ExecuteProcess(
        # cmd=['ign', 'gazebo', 'sim', '-r', '-s', world_file, '--verbose'],
        cmd=['ign', 'gazebo', 'sim', '-r', '-s', world_file],
        output='screen',
        env=env,
        on_exit=Shutdown(),
        condition=UnlessCondition(gui) # Only run if gui is FALSE
    )

    return LaunchDescription([
        gui_arg,
        gazebo_gui,
        gazebo_headless
    ])