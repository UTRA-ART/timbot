from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # --- Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('sim')

    # --- 1. Localization (Odom State) ---
    odom_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('odom_state'),
            '/launch/odom_state.launch.py'
        ]),
        launch_arguments={
            'sim': use_sim_time,
        }.items()
    )

    # --- 2. Vision (ZED Camera) [FUTURE] ---
    # zed_wrapper_launch = IncludeLaunchDescription(...)

    # --- 3. SLAM (Cartographer) [FUTURE] ---
    # cartographer_launch = IncludeLaunchDescription(...)

    # --- 4. Navigation (Nav2) [FUTURE] ---
    # nav2_launch = IncludeLaunchDescription(...)

    return LaunchDescription([
        use_sim_time_arg,
        odom_state_launch,
    ])