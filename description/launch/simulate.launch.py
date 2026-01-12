from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Declare launch arguments
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Use Gazebo GUI'
    )
    
    rqt_steer_arg = DeclareLaunchArgument(
        'rqt_steer',
        default_value='false',
        description='Use RQT steering'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='full',
        description='IGVC world: full, walls, ramp, plain'
    )
    
    world_type_arg = DeclareLaunchArgument(
        'world_type',
        default_value='pavement',
        description='IGVC world type: pavement (2022 IGVC)'
    )
    
    # Get package directories
    gazebo_worlds_share = get_package_share_directory('gazebo_worlds')
    
    # Set GZ_SIM_RESOURCE_PATH for Gazebo to find models and textures
    home_dir = os.path.expanduser('~')
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(gazebo_worlds_share) + ':' + 
              os.path.join(home_dir, '.gazebo/models') + ':' +
              os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )
    
    # World file path (constructed for ExecuteProcess)
    # Note: We use PathJoinSubstitution for the world file
    world_file = PathJoinSubstitution([
        FindPackageShare('gazebo_worlds'),
        LaunchConfiguration('world_type'),
        ['igvc_', LaunchConfiguration('world'), '.world']
    ])
    
    # Ignition Gazebo launch using ros_gz_sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'),
            '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Spawn robot
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('description'),
            '/launch/spawn.launch.py'
        ]),
        launch_arguments={
            'world_type': LaunchConfiguration('world_type')
        }.items()
    )
    
    # Filter lidar data (optional - comment out if package doesn't exist)
    # filter_lidar = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         FindPackageShare('filter_lidar_data'),
    #         '/launch/filter_lidar_data.launch.py'
    #     ])
    # )
    
    # RViz (conditional)
    rviz_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('rviz')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('description'),
                    '/launch/rviz.launch.py'
                ])
            )
        ]
    )
    
    # Ground truth transform
    ground_truth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ground_truth_transform',
        arguments=['--x', '-19.5', '--y', '0', '--z', '0', '--roll', '1.5707', '--pitch', '0', '--yaw', '0', '--frame-id', 'world', '--child-frame-id', 'ground_truth']
    )
    
    # Cartographer (optional - comment out if not needed initially)
    # cartographer = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         FindPackageShare('description'),
    #         '/launch/cartographer.launch.py'
    #     ])
    # )

    return LaunchDescription([
        gz_resource_path,
        use_gui_arg,
        rqt_steer_arg,
        rviz_arg,
        world_arg,
        world_type_arg,
        gazebo,
        spawn_robot,
        # filter_lidar,  # Commented out
        rviz_group,
        ground_truth_tf,
        # cartographer  # Commented out
    ])