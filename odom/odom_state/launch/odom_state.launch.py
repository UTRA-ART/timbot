from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import os

def generate_launch_description():
    launch_arg = DeclareLaunchArgument(
        "launch_state", default_value=TextSubstitution(text="sim")
    )

    odom_state_dir = get_package_share_directory('odom_state')
    odom_local_yaml = os.path.join(odom_state_dir, 'config', 'odom_local.yaml')
    odom_global_yaml = os.path.join(odom_state_dir, 'config', 'odom_global.yaml')
    navsat_yaml = os.path.join(odom_state_dir, 'config', 'navsat.yaml')
    launch_state = LaunchConfiguration('launch_state')

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

    navsat_transform_node = GroupAction(
            condition=IfCondition(
                PythonExpression(["'", launch_state, "' == 'sim'"])
            ),
            actions=[
                Node(
                    package='robot_localization',
                    executable='navsat_transform_node',
                    name='navsat_transform_node',
                    respawn=True,
                    remappings=[('odometry/filtered', 'odometry/global')],
                    parameters=[navsat_yaml]
                )
            ]
        )

    return LaunchDescription([
        launch_arg,
        ekf_local,
        ekf_global,
        navsat_transform_node
        ])