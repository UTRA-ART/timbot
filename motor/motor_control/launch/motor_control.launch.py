from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    
    # Declare launch arguments
    launch_state_arg = DeclareLaunchArgument(
        'launch_state',
        default_value='sim',
        description='Launch state (sim or real)'
    )

    launch_state = LaunchConfiguration('launch_state')
    is_real = PythonExpression(["'", launch_state, "' == 'real'"])

    # Real hardware motor control node
    motor_control_node = Node(
        package='motor_control',
        executable='motor_control.py',
        name='motor_control_node',
        output='screen',
        condition=IfCondition(is_real)
    )

    # Simple echo node for simulation
    motor_echo_node = Node(
        package='motor_control',
        executable='simple_motor_echo.py',
        name='simple_motor_echo',
        output='screen',
        condition=UnlessCondition(is_real)
    )

    return LaunchDescription([
        launch_state_arg,
        motor_control_node,
        motor_echo_node
    ])