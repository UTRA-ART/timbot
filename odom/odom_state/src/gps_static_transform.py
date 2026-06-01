#!/usr/bin/env python3
"""
Static GPS -> map transform + dead-reckoned GPS publisher.

This node mimics the *initial* behavior of navsat_transform_node without
continuously fusing GPS. It:
  1) Latches the first valid GPS fix (or a configured datum).
  2) Latches the latest IMU heading (ENU) to set the map yaw.
  3) Publishes a static TF from `utm` -> `map` based on that initial pose.
  4) Uses local odometry to publish a dead-reckoned NavSatFix (optional).

The static TF lets GPS goals (UTM) transform into the map frame for Nav2.
"""

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float64

import tf2_ros
import tf2_geometry_msgs  # noqa: F401
import utm


def quat_mul(a, b):
    """Hamilton product a (x) b, both (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_to_euler(q):
    """Return roll, pitch, yaw from quaternion (x, y, z, w)."""
    x, y, z, w = q

    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class GpsStaticTransform(Node):
    def __init__(self):
        super().__init__('gps_static_transform')

        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odometry/global')
        self.declare_parameter('gps_fix_topic', '/gps/filtered')
        self.declare_parameter('publish_gps_fix', True)
        self.declare_parameter('publish_gps_odom', False)
        self.declare_parameter('gps_odom_topic', '/odometry/gps')
        self.declare_parameter('publish_gps_heading', True)
        self.declare_parameter('gps_heading_topic', '/gps/heading')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('utm_frame', 'utm')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('sensor_frame', 'gps_link')

        self.declare_parameter('wait_for_datum', False)
        self.declare_parameter('datum', [0.0, 0.0, 0.0])
        self.declare_parameter('zero_altitude', True)
        self.declare_parameter('magnetic_declination_radians', 0.0)
        self.declare_parameter('yaw_offset', 0.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('horizontal_stddev', 5.0)
        self.declare_parameter('vertical_stddev', 5.0)
        self.declare_parameter('use_manual_heading', False)
        self.declare_parameter('manual_heading_deg', 0.0)

        self.gps_topic = self.get_parameter('gps_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.gps_fix_topic = self.get_parameter('gps_fix_topic').value
        self.publish_gps_fix = bool(self.get_parameter('publish_gps_fix').value)
        self.publish_gps_odom = bool(self.get_parameter('publish_gps_odom').value)
        self.gps_odom_topic = self.get_parameter('gps_odom_topic').value
        self.publish_gps_heading = bool(self.get_parameter('publish_gps_heading').value)
        self.gps_heading_topic = self.get_parameter('gps_heading_topic').value

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.utm_frame = self.get_parameter('utm_frame').value
        self.base_link_frame = self.get_parameter('base_link_frame').value
        self.sensor_frame = self.get_parameter('sensor_frame').value

        self.wait_for_datum = bool(self.get_parameter('wait_for_datum').value)
        self.datum_param = self.get_parameter('datum').value
        self.zero_altitude = bool(self.get_parameter('zero_altitude').value)
        self.mag_declination = float(
            self.get_parameter('magnetic_declination_radians').value
        )
        self.yaw_offset = float(self.get_parameter('yaw_offset').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.horiz_stddev = float(self.get_parameter('horizontal_stddev').value)
        self.vert_stddev = float(self.get_parameter('vertical_stddev').value)
        self.use_manual_heading = bool(self.get_parameter('use_manual_heading').value)
        self.manual_heading_deg = float(self.get_parameter('manual_heading_deg').value)

        self.latest_imu: Optional[Imu] = None
        self.latest_odom: Optional[Odometry] = None
        self.initialized = False

        self.datum_utm: Optional[Tuple[float, float, float]] = None
        self.utm_zone = None
        self.utm_letter = None
        self.yaw_enu = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.gps_pub = None
        if self.publish_gps_fix:
            self.gps_pub = self.create_publisher(NavSatFix, self.gps_fix_topic, 10)

        self.gps_odom_pub = None
        if self.publish_gps_odom:
            self.gps_odom_pub = self.create_publisher(Odometry, self.gps_odom_topic, 10)

        self.heading_pub = None
        if self.publish_gps_heading:
            self.heading_pub = self.create_publisher(Float64, self.gps_heading_topic, 10)

        self.create_subscription(
            Imu, self.imu_topic, self._imu_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            NavSatFix, self.gps_topic, self._gps_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, 10
        )

        if self.publish_rate > 0.0:
            self.create_timer(1.0 / self.publish_rate, self._publish_dead_reckoned_gps)

        heading_src = f'manual ({self.manual_heading_deg:.1f} deg)' if self.use_manual_heading else 'IMU'
        self.get_logger().info(
            f'gps_static_transform ready: heading source={heading_src}, waiting for datum to set utm->map'
        )

    def _imu_callback(self, msg: Imu):
        self.latest_imu = msg
        if not self.initialized:
            self._try_initialize()

    def _gps_callback(self, msg: NavSatFix):
        if not self.initialized:
            self._try_initialize(gps_msg=msg)

    def _odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def _try_initialize(self, gps_msg: Optional[NavSatFix] = None):
        if self.initialized:
            return
        # IMU is only required when deriving heading from it.
        if not self.use_manual_heading and self.latest_imu is None:
            return

        datum = self._get_datum_from_params()
        if datum is None:
            if gps_msg is None:
                return
            if gps_msg.status.status == NavSatStatus.STATUS_NO_FIX:
                return
            datum = (gps_msg.latitude, gps_msg.longitude, gps_msg.altitude)

        lat, lon, alt = datum
        if alt is None:
            if gps_msg is not None and gps_msg.status.status != NavSatStatus.STATUS_NO_FIX:
                alt = gps_msg.altitude
            else:
                alt = 0.0
        utm_coords = utm.from_latlon(lat, lon)
        easting, northing, zone, letter = utm_coords

        if self.zero_altitude:
            alt = 0.0

        self.datum_utm = (float(easting), float(northing), float(alt))
        self.utm_zone = zone
        self.utm_letter = letter

        if self.use_manual_heading:
            self.yaw_enu = math.radians(self.manual_heading_deg)
            heading_src = f'manual ({self.manual_heading_deg:.2f} deg)'
        else:
            self.yaw_enu = self._get_imu_yaw(self.latest_imu)
            heading_src = f'IMU ({math.degrees(self.yaw_enu):.2f} deg)'

        self._broadcast_static_tf()

        self.initialized = True
        self.get_logger().info(
            f'UTM->map static TF set: datum=({lat:.7f}, {lon:.7f}, {alt:.2f}), '
            f'utm=({easting:.2f}, {northing:.2f}, zone {zone}{letter}), '
            f'heading={heading_src}'
        )

    def _get_datum_from_params(self) -> Optional[Tuple[float, float, Optional[float]]]:
        if not self.wait_for_datum:
            return None

        datum = self.datum_param
        if isinstance(datum, str):
            # Accept strings like "[lat, lon, alt]" from LaunchConfiguration
            cleaned = datum.strip().lstrip('[').rstrip(']')
            parts = [p.strip() for p in cleaned.split(',') if p.strip()]
            if len(parts) not in (2, 3):
                return None
            try:
                datum = [float(p) for p in parts]
            except ValueError:
                return None

        if not isinstance(datum, (list, tuple)) or len(datum) not in (2, 3):
            return None

        lat = float(datum[0])
        lon = float(datum[1])
        alt = float(datum[2]) if len(datum) == 3 else None
        if lat == 0.0 and lon == 0.0:
            return None
        return lat, lon, alt

    def _get_imu_yaw(self, imu_msg: Imu) -> float:
        q = (
            imu_msg.orientation.x,
            imu_msg.orientation.y,
            imu_msg.orientation.z,
            imu_msg.orientation.w,
        )
        if q == (0.0, 0.0, 0.0, 0.0):
            return 0.0
        _, _, yaw = quat_to_euler(q)
        yaw += self.mag_declination
        yaw += self.yaw_offset
        return self._normalize_angle(yaw)

    def _broadcast_static_tf(self):
        if self.datum_utm is None:
            return

        easting, northing, alt = self.datum_utm
        half = self.yaw_enu / 2.0
        qz = math.sin(half)
        qw = math.cos(half)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.utm_frame
        tf_msg.child_frame_id = self.map_frame
        tf_msg.transform.translation.x = easting
        tf_msg.transform.translation.y = northing
        tf_msg.transform.translation.z = alt
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.static_broadcaster.sendTransform(tf_msg)

    def _publish_dead_reckoned_gps(self):
        # Heading is published independently of GPS/odom init — just needs IMU.
        # ENU convention: East=0°, North=90°, West=±180°, South=-90°.
        # Facing true north → 90° when magnetic_declination_radians is correct.
        if self.heading_pub is not None and self.latest_imu is not None:
            yaw_rad = self._get_imu_yaw(self.latest_imu)
            msg = Float64()
            msg.data = math.degrees(yaw_rad)
            self.heading_pub.publish(msg)

        # When use_manual_heading is set there is no IMU callback driving init,
        # so retry here on every timer tick until the datum is ready.
        if not self.initialized and self.use_manual_heading:
            self._try_initialize()

        if not self.initialized:
            self.get_logger().warn('dead_reckoned_gps: not initialized')
            return
        if self.latest_odom is None:
            self.get_logger().warn('dead_reckoned_gps: no /odometry/local yet')
            return

        map_pose = self._odom_pose_in_map(self.latest_odom)
        if map_pose is None:
            return

        if self.datum_utm is None:
            self.get_logger().warn('dead_reckoned_gps: datum UTM not set')
            return

        easting, northing, alt = self._map_pose_to_utm(map_pose)
        lat, lon = utm.to_latlon(easting, northing, self.utm_zone, self.utm_letter)

        if self.publish_gps_fix and self.gps_pub is not None:
            fix = NavSatFix()
            fix.header.stamp = self.get_clock().now().to_msg()
            fix.header.frame_id = self.sensor_frame
            fix.status.status = NavSatStatus.STATUS_FIX
            fix.status.service = NavSatStatus.SERVICE_GPS
            fix.latitude = float(lat)
            fix.longitude = float(lon)
            fix.altitude = float(alt)
            fix.position_covariance = [
                self.horiz_stddev ** 2, 0.0, 0.0,
                0.0, self.horiz_stddev ** 2, 0.0,
                0.0, 0.0, self.vert_stddev ** 2,
            ]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.gps_pub.publish(fix)

        if self.publish_gps_odom and self.gps_odom_pub is not None:
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = self.utm_frame
            odom.child_frame_id = self.base_link_frame
            odom.pose.pose.position.x = float(easting)
            odom.pose.pose.position.y = float(northing)
            odom.pose.pose.position.z = float(alt)

            q_map = (
                map_pose.pose.orientation.x,
                map_pose.pose.orientation.y,
                map_pose.pose.orientation.z,
                map_pose.pose.orientation.w,
            )
            half = self.yaw_enu / 2.0
            q_utm = quat_mul((0.0, 0.0, math.sin(half), math.cos(half)), q_map)
            odom.pose.pose.orientation.x = q_utm[0]
            odom.pose.pose.orientation.y = q_utm[1]
            odom.pose.pose.orientation.z = q_utm[2]
            odom.pose.pose.orientation.w = q_utm[3]
            odom.pose.covariance = [0.0] * 36
            odom.pose.covariance[0] = self.horiz_stddev ** 2
            odom.pose.covariance[7] = self.horiz_stddev ** 2
            odom.pose.covariance[14] = self.vert_stddev ** 2
            self.gps_odom_pub.publish(odom)

    def _odom_pose_in_map(self, odom_msg: Odometry) -> Optional[PoseStamped]:
        pose = PoseStamped()
        pose.header = odom_msg.header
        pose.pose = odom_msg.pose.pose

        if pose.header.frame_id == self.map_frame:
            return pose

        # Use Time(0) = "latest available" to avoid ExtrapolationException
        # when the odom timestamp is slightly ahead of the newest TF data.
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0

        try:
            return self.tf_buffer.transform(pose, self.map_frame, timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().error(f'TF lookup odom->map failed: {type(e).__name__}: {e}')
            return None

    def _map_pose_to_utm(self, map_pose: PoseStamped) -> Tuple[float, float, float]:
        if self.datum_utm is None:
            return 0.0, 0.0, 0.0

        x = map_pose.pose.position.x
        y = map_pose.pose.position.y
        z = map_pose.pose.position.z

        cos_yaw = math.cos(self.yaw_enu)
        sin_yaw = math.sin(self.yaw_enu)
        dx = cos_yaw * x - sin_yaw * y
        dy = sin_yaw * x + cos_yaw * y

        e0, n0, a0 = self.datum_utm
        return e0 + dx, n0 + dy, a0 + z

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = GpsStaticTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
