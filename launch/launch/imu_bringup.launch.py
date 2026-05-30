"""
IMU Bringup (real rover)
========================
Brings up the Phidgets Spatial as a RAW sensor source plus an
``imu_filter_madgwick`` node that fuses accel + gyro + magnetometer into an
ENU-referenced orientation on ``/imu/data``.

Why this exists
---------------
The Phidgets onboard AHRS reports its orientation quaternion in a NED-style
(Z-down, north-referenced) frame, which is incompatible with ROS/REP-103 (ENU).
Instead of using that quaternion, we run the driver with ``use_orientation:=False``
(so it publishes only raw ``/imu/data_raw`` + ``/imu/mag``) and let
``imu_filter_madgwick`` compute orientation directly in ENU
(``world_frame:=enu``).  Nothing downstream ever sees NED.

Topic flow:
    phidgets_spatial -> /imu/data_raw (accel+gyro), /imu/mag (magnetometer)
    imu_filter_madgwick (/imu/data_raw + /imu/mag) -> /imu/data (ENU orientation)

PREREQUISITE: the magnetometer must be calibrated for heading to be correct.
Fill in the ``cc_*`` compass-correction params below once calibrated (see the
Phidgets 1044 user guide); until then heading from the mag will wander.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Use simulation clock if true (real rover: false)',
    )
    use_sim_time = LaunchConfiguration('sim')

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error',
    )
    log_level = LaunchConfiguration('log_level')

    # --- Phidgets Spatial: RAW data only (no onboard AHRS orientation) ---
    phidgets_params = {
        # Do NOT use the onboard AHRS — its quaternion is NED. madgwick gives ENU.
        'use_orientation': False,
        'frame_id': 'imu_link',
        # 125 Hz is the magnetometer ceiling; matches the URDF imu_update_rate.
        'data_interval_ms': 8,
        'publish_rate': 0.0,           # publish on every device sample
        'use_sim_time': use_sim_time,

        # --- Magnetometer compass-correction (hard/soft-iron) calibration ---
        # From the Phidget Control Panel magnetometer calibration (run with the IMU
        # mounted on the powered robot). These are ALSO persisted in device firmware;
        # setting them here just documents the calibration in git and restores it if
        # firmware is ever reset. Idempotent with firmware (it replaces, not stacks).
        # NOTE: do NOT also set madgwick mag_bias_* — that would double-correct.
        # Order: magField, offset0-2, gain0-2, T0-5.
        'cc_mag_field': 1.00000,
        'cc_offset0': 0.78557, 'cc_offset1': 0.63800, 'cc_offset2': -0.41549,
        'cc_gain0': 0.97048,  'cc_gain1': 1.43722,  'cc_gain2': 1.49328,
        'cc_t0': -0.61325, 'cc_t1': 0.16235, 'cc_t2': -0.76681,
        'cc_t3': -0.22019, 'cc_t4': 0.56628, 'cc_t5': 0.33753,
    }

    phidgets_container = ComposableNodeContainer(
        name='phidget_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='phidgets_spatial',
                plugin='phidgets::SpatialRosI',
                name='phidgets_spatial',
                parameters=[phidgets_params],
            ),
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )

    # --- Madgwick filter: raw accel/gyro/mag -> ENU orientation on /imu/data ---
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag': True,            # fuse magnetometer for absolute heading
            'world_frame': 'enu',       # <-- produces REP-103 ENU orientation
            'publish_tf': False,        # ekf owns the TF tree, not this filter
            'gain': 0.1,
            'use_sim_time': use_sim_time,
        }],
        # Subscribes /imu/data_raw + /imu/mag, publishes /imu/data (all global ns).
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        sim_arg,
        log_level_arg,
        phidgets_container,
        imu_filter,
    ])
