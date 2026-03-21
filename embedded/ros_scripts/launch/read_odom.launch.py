from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Declare launch arguments
    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port',
        default_value='/dev/ttyACM0',
        description='Serial port for the Arduino'
    )
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    ros_rate_arg = DeclareLaunchArgument(
        'ros_rate',
        default_value='30',
        description='ROS loop rate in Hz'
    )

    # Read odometry from Arduino node
    read_odom_node = Node(
        package='ros_scripts',
        executable='read_odom_arduino.py',
        name='ticks_publisher',
        output='screen',
        parameters=[{
            'arduino_port': LaunchConfiguration('arduino_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'ros_rate': LaunchConfiguration('ros_rate'),
        }]
    )

    return LaunchDescription([
        arduino_port_arg,
        baud_rate_arg,
        ros_rate_arg,
        read_odom_node,
    ])
