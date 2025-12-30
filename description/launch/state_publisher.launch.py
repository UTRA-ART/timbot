from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Parse URDF with xacro
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('description'),
                'rover_model',
                'urdf',
                'espresso.urdf.xacro'
            ])
        ]),
        value_type=str
    )
    
    robot_description = {'robot_description': robot_description_content}
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'rate': 50},
            robot_description
        ]
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description]
    )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher
    ])