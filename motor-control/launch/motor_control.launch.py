from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    
    # Declare launch arguments
    launch_state_arg = DeclareLaunchArgument(
        'launch_state',
        default_value='sim',
        description='Launch state (sim or real)'
    )
    
    # Simple echo node that just echoes cmd_vel
    motor_echo_node = Node(
        package='motor_control',
        executable='simple_motor_echo.py',
        name='simple_motor_echo',
        output='screen'
    )

    return LaunchDescription([
        launch_state_arg,
        motor_echo_node
    ])