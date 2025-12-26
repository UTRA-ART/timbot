from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Get URDF via xacro
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("description"),
                "rover_model",
                "urdf", 
                "espresso.urdf.xacro"
            ])
        ]),
        value_type=str
    )
    
    robot_description = {"robot_description": robot_description_content}

    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_ros"), 
            "/launch/gazebo.launch.py"
        ]),
        launch_arguments={"verbose": "false"}.items()
    )

    # Static transform publisher
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_footprint_base",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    # Spawn entity
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "espresso", "-x", "0", "-y", "0", "-z", "5"],
        output="screen"
    )

    # Fake joint calibration publisher
    fake_joint_calibration = ExecuteProcess(
        cmd=["ros2", "topic", "pub", "/calibrated", "std_msgs/msg/Bool", "data: true", "-1"],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        static_tf,
        robot_state_publisher,
        spawn_entity,
        fake_joint_calibration
    ])