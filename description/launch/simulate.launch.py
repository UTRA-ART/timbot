from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
    
    # World file path
    world_file = PathJoinSubstitution([
        FindPackageShare('gazebo_worlds'),
        LaunchConfiguration('world_type'),
        ['igvc_', LaunchConfiguration('world'), '.world']
    ])
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': world_file,
            'paused': 'false',
            'verbose': 'false',
            'use_sim_time': 'true',
            'gui': LaunchConfiguration('use_gui'),
            'debug': 'false',
            'server_required': 'false'
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
    
    # Filter lidar data
    filter_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('filter_lidar_data'),
            '/launch/filter_lidar_data.launch.py'
        ])
    )
    
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
        arguments=['-19.5', '0', '0', '1.5707', '0', '0', 'world', 'ground_truth']
    )
    
    # Cartographer
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('description'),
            '/launch/cartographer.launch.py'
        ])
    )

    return LaunchDescription([
        use_gui_arg,
        rqt_steer_arg,
        rviz_arg,
        world_arg,
        world_type_arg,
        gazebo,
        spawn_robot,
        filter_lidar,
        rviz_group,
        ground_truth_tf,
        cartographer
    ])