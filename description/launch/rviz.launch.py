from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    # Path to RViz configuration file
    rviz_config = PathJoinSubstitution([
        FindPackageShare("description"),
        "rviz",
        "timbot.rviz"
    ])
    
    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config],
        output="screen"
    )

    return LaunchDescription([
        rviz_node
    ])