from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # --- Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('sim')

    # --- Waypoints File Selection ---
    # Get package share directory for waypoints
    pkg_share = get_package_share_directory('load_waypoints')

    # Select waypoints file based on sim argument
    # sim=true -> pavement waypoints, sim=false -> IGVC course
    waypoints_file = PythonExpression([
        "'", os.path.join(pkg_share, 'jsons', 'sim_waypoints.json'), "' if '",
        use_sim_time, "' == 'true' else '",
        os.path.join(pkg_share, 'jsons', 'IGVC_course.json'), "'"
    ])

    # --- 1. Waypoint Navigation Server ---
    # Loads JSON waypoints and navigates through them sequentially
    navigate_waypoints_node = Node(
        package='load_waypoints',
        executable='navigate_waypoints.py',
        name='load_waypoints_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'waypoints_file': waypoints_file
        }]
    )

    # --- 2. Ramp Navigation ---
    # Detects ramps and handles crossing safely
    ramp_navigate_node = Node(
        package='load_waypoints',
        executable='ramp_navigate.py',
        name='ramp_navigate',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- 3. Navigation Options Service ---
    # Provides /rover_navigation service for manual goal commands
    nav_options_node = Node(
        package='load_waypoints',
        executable='nav_options.py',
        name='nav_control',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        navigate_waypoints_node,
        ramp_navigate_node,
        nav_options_node,
    ])