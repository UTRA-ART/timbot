from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    # Declare launch arguments
    launch_state_arg = DeclareLaunchArgument(
        'launch_state',
        default_value='sim',
        description='Launch state (sim or real)'
    )
    
    # Use your existing timbot_twist_mux
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('timbot_twist_mux'),
                'launch',
                'twist_mux.launch.py'
            ])
        ])
    )
    
    # Motor controller + feedback launch (if you create this package)
    motor_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('motor_control'),
                'launch',
                'motor_control.launch.py'
            ])
        ]),
        launch_arguments={
            'launch_state': LaunchConfiguration('launch_state')
        }.items()
    )

    return LaunchDescription([
        launch_state_arg,
        twist_mux_launch,
        motor_control_launch
    ])