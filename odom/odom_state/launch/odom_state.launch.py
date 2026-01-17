from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    launch_arg = DeclareLaunchArgument(
        "launch_state", default_value=TextSubstitution(text="sim")
    )

    odom_state_dir = get_package_share_directory('odom_state')
    odom_local_yaml = os.path.join(odom_state_dir, 'config', 'odom_local.yaml')
    odom_global_yaml = os.path.join(odom_state_dir, 'config', 'odom_global.yaml')
    navsat_yaml = os.path.join(odom_state_dir, 'config', 'navsat.yaml')

    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        remappings=[("odometry/filtered", "odometry/local")],
        parameters=[odom_local_yaml]
    )

    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        remappings=[('odometry/filtered', 'odometry/global')],
        parameters=[odom_global_yaml]
    )

    # navsat_transform_node always runs - GPS available in all modes:
    # - 'real': hardware GPS on rover
    # - 'standalone': fake_sensor_publisher provides simulated GPS
    # - 'sim': Gazebo GPS plugin provides simulated GPS
    # Note: reads from ekf_local to avoid circular dependency with ekf_global
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        respawn=True,
        remappings=[('odometry/filtered', 'odometry/local')],
        parameters=[navsat_yaml]
    )

    return LaunchDescription([
        launch_arg,
        ekf_local,
        ekf_global,
        navsat_transform_node
    ])
       