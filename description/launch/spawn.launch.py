"""
Gazebo Spawn Launch
===================
Sim-only launch file that creates the Gazebo ↔ ROS bridges and spawns the
timbot entity in the simulator.

Robot State Publisher, Joint State Publisher, and Twist Mux have been moved
to robot_bringup.launch.py so they can be shared between sim and real modes.
The Robot Bringup stage must be launched BEFORE this stage so that
/robot_description is available for spawn_robot to read.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # --- Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('sim')

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error'
    )
    log_level = LaunchConfiguration('log_level')

    # Spawn position arguments
    x_arg = DeclareLaunchArgument('x', default_value='-19.5')
    y_arg = DeclareLaunchArgument('y', default_value='0')
    z_arg = DeclareLaunchArgument('z', default_value='0.05')
    roll_arg = DeclareLaunchArgument('roll', default_value='0')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='1.5708')

    # Camera enable — controls whether the camera bridge is started
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='false',
        description='Bridge ZED camera topics from Gazebo'
    )
    enable_camera = LaunchConfiguration('enable_camera')

    # --- Nodes ---

    # 1. ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Bridge the Clock (so ROS knows the sim time)
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',

            # Bridge the Joint States (so ROS knows where the wheels are)
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',

            # Bridge the Lidar
            '/scan_lower@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/scan_upper@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',

            # Bridge the GPS (gps_cov_relay adds covariance → /gps/fix_cov)
            '/gps/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',

            # Bridge the IMU
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',

            # Allow ROS to send drive commands TO Gazebo (Note the ']')
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # Gazebo set pose, delete and creation of entity
            '/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            '/world/default/delete@ros_gz_interfaces/srv/DeleteEntity',
            '/world/default/create@ros_gz_interfaces/srv/SpawnEntity',
            # Receive raw odometry FROM Gazebo (Optional, but good for debugging)
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '--ros-args', '--log-level', log_level,
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 2. Spawn Robot entity in Gazebo
    # Reads URDF from /robot_description topic (published by RSP in robot_bringup)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_timbot',
        arguments=[
            '-name', 'timbot',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-R', LaunchConfiguration('roll'),
            '-P', LaunchConfiguration('pitch'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3. Camera Bridge — only launched when enable_camera is true
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        condition=IfCondition(enable_camera),
        arguments=[
            '/zed_node/left/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed_node/left/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed_node/left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/zed_node/left/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '--ros-args', '--log-level', log_level,
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        x_arg, y_arg, z_arg, roll_arg, pitch_arg, yaw_arg,
        enable_camera_arg,
        bridge,
        camera_bridge,
        spawn_robot,
    ])