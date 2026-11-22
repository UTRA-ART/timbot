from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_topic = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odometry/global",
    )
    plan_topic = DeclareLaunchArgument(
        "plan_topic",
        default_value="/plan",
    )
    cmd_vel_topic = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/nav_vel",
    )
    controller_frequency = DeclareLaunchArgument(
        "controller_frequency",
        default_value="10.0",
    )
    lookahead_distance = DeclareLaunchArgument(
        "lookahead_distance", default_value="0.2"
    )
    goal_tolerance = DeclareLaunchArgument(
        "goal_tolerance", default_value="0.05"
    )
    kP = DeclareLaunchArgument("kP", default_value="10.5")
    kD = DeclareLaunchArgument("kD", default_value="0.1")
    max_linear_velocity = DeclareLaunchArgument(
        "max_linear_velocity", default_value="0.3"
    )
    max_angular_velocity = DeclareLaunchArgument(
        "max_angular_velocity", default_value="1.5"
    )
    max_forwards_linear_acceleration = DeclareLaunchArgument(
        "max_forwards_linear_acceleration", default_value="1.0"
    )
    max_angular_acceleration = DeclareLaunchArgument(
        "max_angular_acceleration", default_value="2.0"
    )

    controller = Node(
        package="nav_stack",
        executable="custom_nav_controller",
        name="custom_nav_controller",
        output="screen",
        parameters=[
            {
                "odom_topic": LaunchConfiguration("odom_topic"),
                "plan_topic": LaunchConfiguration("plan_topic"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "controller_frequency": LaunchConfiguration(
                    "controller_frequency"
                ),
                "lookahead_distance": LaunchConfiguration("lookahead_distance"),
                "goal_tolerance": LaunchConfiguration("goal_tolerance"),
                "kP": LaunchConfiguration("kP"),
                "kD": LaunchConfiguration("kD"),
                "max_linear_velocity": LaunchConfiguration(
                    "max_linear_velocity"
                ),
                "max_angular_velocity": LaunchConfiguration(
                    "max_angular_velocity"
                ),
                "max_forwards_linear_acceleration": LaunchConfiguration(
                    "max_forwards_linear_acceleration"
                ),
                "max_angular_acceleration": LaunchConfiguration(
                    "max_angular_acceleration"
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            odom_topic,
            plan_topic,
            cmd_vel_topic,
            controller_frequency,
            lookahead_distance,
            goal_tolerance,
            kP,
            kD,
            max_linear_velocity,
            max_angular_velocity,
            max_forwards_linear_acceleration,
            max_angular_acceleration,
            controller,
        ]
    )
