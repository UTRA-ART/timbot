import sys
from launch import LaunchDescription, LaunchContext
from launch.actions import ExecuteProcess, Shutdown, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import shutil


def _patched_rendering_media(pkg_share: str) -> str:
    """Return a copy of ign-rendering's media dir with rendering/ laid over it.

    rendering/ holds a fixed ogre gpu_lidar shader: the stock one draws each ray
    as a point on a pixel corner, which WSL's GPU driver drops (every ray then
    reads range_min). Gazebo only reads media from one directory, hence the copy.
    """
    dst = os.path.join(os.path.expanduser('~'), '.cache', 'timbot', 'ignition-rendering6')
    src = os.environ.get('IGN_RENDERING_RESOURCE_PATH', '/usr/share/ignition/ignition-rendering6')
    if os.path.realpath(src) != os.path.realpath(dst):
        shutil.copytree(src, dst, dirs_exist_ok=True)
    shutil.copytree(os.path.join(pkg_share, 'rendering'), dst, dirs_exist_ok=True)
    return dst


def _setup_gazebo(context: LaunchContext):
    """Resolve world_file at launch time and return Gazebo process actions."""
    world_file_name = context.launch_configurations.get('world_file', 'track.world')

    # Resolve world file path from installed worlds/ directory
    pkg_share = get_package_share_directory('gazebo_worlds')
    world_file = os.path.join(pkg_share, 'worlds', world_file_name)

    if not os.path.exists(world_file):
        available = [f for f in os.listdir(os.path.join(pkg_share, 'worlds')) if f.endswith('.world')]
        raise FileNotFoundError(
            f"World file not found: {world_file}\n"
            f"Available worlds: {available}"
        )

    # Set Gazebo resource path to find models in workspace
    home_dir = os.path.expanduser('~')
    gazebo_models_path = os.path.join(home_dir, '.gazebo/models')
    worlds_dir = os.path.join(pkg_share, 'worlds')
    description_resource_path = os.path.dirname(get_package_share_directory('description'))

    env = os.environ.copy()
    env['IGN_GAZEBO_RESOURCE_PATH'] = f"{gazebo_models_path}:{worlds_dir}:{description_resource_path}"
    env['LIBGL_ALWAYS_SOFTWARE'] = context.launch_configurations.get('software_rendering', '1')
    env['IGN_RENDERING_RESOURCE_PATH'] = _patched_rendering_media(pkg_share)

    # Full GUI. The GUI defaults to ogre2, which crashes on WSL's GPU driver.
    gazebo_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', 'sim', '-r', '--render-engine-gui', 'ogre', world_file],
        output='screen',
        env=env,
        on_exit=Shutdown(),
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Headless / Server Only
    gazebo_headless = ExecuteProcess(
        cmd=['ign', 'gazebo', 'sim', '-r', '-s', world_file],
        output='screen',
        env=env,
        on_exit=Shutdown(),
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    return [gazebo_gui, gazebo_headless]


def generate_launch_description():
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run Gazebo with GUI (true) or headless (false)'
    )

    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='track.world',
        description='Name of the .world file in gazebo_worlds/worlds/'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error'
    )

    return LaunchDescription([
        gui_arg,
        world_file_arg,
        log_level_arg,
        OpaqueFunction(function=_setup_gazebo),
    ])