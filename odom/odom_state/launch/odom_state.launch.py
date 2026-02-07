from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # 1. Declare the use_sim_time argument (Default to true for safety in this context)
    use_sim_time_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('sim')

    # 2. Derive launch_state automatically
    # If use_sim_time is true, state is 'sim'. Otherwise 'real'.
    launch_state = PythonExpression([
        "'sim' if '", use_sim_time, "' == 'true' else 'real'"
    ])

    # 3. Path Setup
    odom_state_dir = get_package_share_directory('odom_state')
    odom_local_yaml = os.path.join(odom_state_dir, 'config', 'odom_local.yaml')
    odom_global_yaml = os.path.join(odom_state_dir, 'config', 'odom_global.yaml')
    navsat_yaml = os.path.join(odom_state_dir, 'config', 'navsat.yaml')

    # 4. Nodes (Now receiving the time parameter!)
    
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        remappings=[('/odometry/filtered', '/odometry/local')],
        parameters=[
            odom_local_yaml, 
            {'use_sim_time': use_sim_time}, 
            {'launch_state': launch_state} 
        ]
    )

    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        remappings=[('/odometry/filtered', '/odometry/global')],
        parameters=[
            odom_global_yaml,
            {'use_sim_time': use_sim_time},
            {'launch_state': launch_state}
        ]
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        respawn=False,
        remappings=[('/odometry/filtered', '/odometry/local')],
        parameters=[
            navsat_yaml,
            {'use_sim_time': use_sim_time},
            {'launch_state': launch_state}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_local,
        ekf_global,
        navsat_transform_node
    ])