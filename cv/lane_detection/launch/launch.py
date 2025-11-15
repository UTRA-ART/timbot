# Run with: `ros2 launch lane_detection lane_detection.launch.py launch_state:=real`
# See: https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html#id2
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # TODO: We previously used launch_state to switch the zed camera topic image is mapped to.
    # It was commented out, so it's unclear if it's still needed. We will use RGB for now and see if any issues arise.
    launch_state = LaunchConfiguration('launch_state')

    # Get parameters for lane detection from its yaml file
    lane_detection_params = PathJoinSubstitution([
        # FindPackageShare is used to get the path to the package, which it knows from when you built with colcon
        FindPackageShare('lane_detection'),'config','lane_detection_params.yaml'
    ])

    # Runs lane detection server
    lane_detection_inference = Node(
        package='lane_detection',                                   # Name of package
        executable='lane_detection_inference',                      # References lane_detection_inference.py
        name='lane_detection_inference',                            # Name of the node
        output='screen',
        parameters=[lane_detection_params],                         # Custom parameters
        remappings=[('image', '/zed_node/rgb/image_rect_color')],   # Remap input image topic to ZED camera topic
    )

    # Visualizes lane detection results
    lane_viz = Node(
        package='lane_detection',
        executable='lane_viz',
        name='lane_viz',
        output='screen'
    )

    # Scans lane detection results
    lane_scan_cpp = Node(
        package='lane_detection',
        executable='scan',
        name='lane_scan_cpp',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('launch_state', default_value='sim'), # Declare launch argument with default value 'sim'
        lane_detection_inference,                                   # Add lane detection inference node to launch description
        lane_viz,                                                   # Add lane visualization node to launch description
        lane_scan_cpp                                               # Add lane scan node to launch description
    ])