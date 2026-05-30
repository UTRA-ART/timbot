"""
IMU Bringup (real rover) — onboard AHRS + NED→ENU relay
=======================================================
Brings up the Phidgets Spatial with its ONBOARD AHRS enabled
(``use_orientation:=True``), which fuses accel + gyro + magnetometer on-device
into an orientation quaternion — but in a NED-referenced world frame. A small
relay (``odom_state/imu_relay.py``) converts that orientation to ENU (REP-103)
and republishes ``/imu/data``.

Topic flow:
    phidgets_spatial (AHRS) -> /imu/data_raw  (NED orientation + accel + gyro)
    imu_relay (/imu/data_raw) -> /imu/data    (ENU orientation, accel/gyro passthrough)
    phidgets_spatial         -> /imu/mag      (unused downstream; AHRS uses mag on-device)

Why this instead of imu_filter_madgwick:
    The onboard AHRS handles the device's internal sensor-axis conventions and
    mag fusion itself, so we only need a single fixed NED→ENU world rotation in
    the relay rather than feeding raw axes to an external filter.

PREREQUISITE: heading accuracy still depends on a calibrated magnetometer. The
AHRS uses the magnetometer for yaw, and the robot's own magnetic field (motors,
battery, steel) distorts it. Calibrate each device once via the Phidget Control
Panel (mounted on the powered robot); the correction persists in the device's
firmware. The relay/AHRS choice does NOT remove this requirement.
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

    # --- Phidgets Spatial with onboard AHRS (orientation in NED) ---
    phidgets_params = {
        'use_orientation': True,        # enable on-device AHRS fusion
        'spatial_algorithm': 'ahrs',
        'frame_id': 'imu_link',
        # 125 Hz is the magnetometer ceiling; matches the URDF imu_update_rate.
        'data_interval_ms': 8,
        'publish_rate': 0.0,            # publish on every device sample
        'use_sim_time': use_sim_time,

        # AHRS tuning (Phidgets stock defaults).
        'ahrs_angular_velocity_threshold': 1.0,
        'ahrs_angular_velocity_delta_threshold': 0.1,
        'ahrs_acceleration_threshold': 0.1,
        'ahrs_mag_time': 10.0,
        'ahrs_accel_time': 10.0,
        'ahrs_bias_time': 1.25,

        # --- Magnetometer compass-correction (hard/soft-iron) calibration ---
        # IMPORTANT: calibration is PER-DEVICE and persists in each unit's firmware
        # (run the Phidget Control Panel magnetometer calibration once per device,
        # mounted on the powered robot). Do NOT hardcode one device's values here —
        # that would push the wrong correction onto a different unit. Leave these
        # commented so each device uses its own firmware calibration.
        #
        # Reference (device serial ____, calibrated YYYY-MM-DD): magField, offset0-2,
        # gain0-2, T0-5 = 1.00000, 0.78557, 0.63800, -0.41549, 0.97048, 1.43722,
        # 1.49328, -0.61325, 0.16235, -0.76681, -0.22019, 0.56628, 0.33753
        #
        # 'cc_mag_field': ...,
        # 'cc_offset0': ..., 'cc_offset1': ..., 'cc_offset2': ...,
        # 'cc_gain0': ...,  'cc_gain1': ...,  'cc_gain2': ...,
        # 'cc_t0': ..., 'cc_t1': ..., 'cc_t2': ...,
        # 'cc_t3': ..., 'cc_t4': ..., 'cc_t5': ...,
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

    # --- Relay: convert AHRS NED orientation -> ENU, republish /imu/data ---
    imu_relay = Node(
        package='odom_state',
        executable='imu_relay.py',
        name='imu_relay',
        output='screen',
        parameters=[{
            'input_topic': '/imu/data_raw',
            'output_topic': '/imu/data',
            # Set if bench verification shows a constant heading offset (else 0).
            'extra_yaw_offset_deg': 0.0,
            'orientation_stddev': 0.05,
            'use_sim_time': use_sim_time,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        sim_arg,
        log_level_arg,
        phidgets_container,
        imu_relay,
    ])
