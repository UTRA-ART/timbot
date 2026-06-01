#!/usr/bin/env python3
"""
Navigate Waypoints Node

Loads GPS waypoints from a JSON file and navigates through them sequentially
using Nav2. Coordinates with ramp_navigate for ramp crossing.

Also handles quick restart of the nav2 nodes.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from ament_index_python.packages import get_package_share_directory
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs  # Required to register geometry_msgs types with tf2

from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from load_waypoints.srv import RoverRespawn
from robot_localization.srv import SetPose
from cartographer_ros_msgs.srv import StartTrajectory, FinishTrajectory

from geometry_msgs.msg import PoseWithCovarianceStamped

from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity

from nav2_msgs.srv import ClearEntireCostmap

import json
import os
import sys
import time
import threading as th
import utm

from std_srvs.srv import Trigger

class NavigateWaypoints(Node):
    def __init__(self):
        super().__init__('load_waypoints_server')

      
        # Get waypoints file from parameter (set by launch file)
        self.declare_parameter('waypoints_file', '')
        self.waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value

        if not self.waypoints_file:
            self.get_logger().error('No waypoints_file parameter provided!')
            sys.exit(1)
        
        
        self.srv_cb_group = ReentrantCallbackGroup()
        self.client_cb_group = ReentrantCallbackGroup()
        # Callback group for action client (reentrant to allow concurrent callbacks)
        self.action_cb_group = ReentrantCallbackGroup()

        self.set_pose_client = self.create_client(
            SetEntityPose, '/world/default/set_pose',
            callback_group=self.client_cb_group
        )

        self.waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        self.respawn_service = self.create_service(
            RoverRespawn, 'rover_respawn', self.handle_respawn_request,
            callback_group=self.srv_cb_group
        )
        self.list_waypoints_service = self.create_service(
            Trigger, 'list_waypoints', self.handle_list_waypoints_request,
            callback_group=self.srv_cb_group
        )
      
        
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap',
            callback_group=self.client_cb_group
        )
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap',
            callback_group=self.client_cb_group
        )
        self.ekf_local_set_pose_client = self.create_client(
            SetPose, '/ekf_local/set_pose', callback_group=self.client_cb_group
        )
        self.finish_trajectory_client = self.create_client(
            FinishTrajectory, '/finish_trajectory', callback_group=self.client_cb_group
        )
        self.start_trajectory_client = self.create_client(
            StartTrajectory, '/start_trajectory', callback_group=self.client_cb_group
        )

        self.declare_parameter(
            'cartographer_config_dir',
            os.path.join(get_package_share_directory('description'), 'config')
        )

        self.declare_parameter('cartographer_config_basename', 'cartographer.lua')
        self.declare_parameter('cartographer_active_trajectory_id', 0)
        # self.declare_parameter('restart_cartographer_on_respawn', True)
        self.cartographer_config_dir = self.get_parameter('cartographer_config_dir').value
        self.cartographer_config_basename = self.get_parameter('cartographer_config_basename').value
        self.cartographer_active_trajectory_id = int(self.get_parameter('cartographer_active_trajectory_id').value)
        # self.restart_cartographer_on_respawn = bool(self.get_parameter('restart_cartographer_on_respawn').value)
            

        # State
        self.waypoints = {}
        self.pose_queue = []  # Queue of (PoseStamped, description) tuples, in map frame
        self.max_time_for_transform = 60.0
        self.waited_for_transform = False
        self.ignore_lidar = False
        self.start_direction = 1 # going clockwise (1)
        self.laps = 0
        self.current_lap = 0
        self.curr_waypoint_idx = 0
        self.result_received = 0
        self.ramp_naving = False
        self.cv_ramp_naving = th.Condition()
        self.respawning = False
        self.goal_done_event = th.Event()
        self.goal_result = None
        self.cv_respawning = th.Condition()
        self.stop_waypoint_loop = False
        self.is_clockwise = True



        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.waypoint_pub = self.create_publisher(Bool, '/waypoint_int', 10)

        # Subscribers
        self.ramp_naving_sub = self.create_subscription(
            Bool, '/ramp_naving', self.ramp_naving_callback, 10,
            callback_group=self.srv_cb_group
        )
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, 10,
            callback_group=self.srv_cb_group
        )

        # Load waypoints and convert to poses
        self.populate_waypoint_dict()

        # Set initial waypoint index based on direction
        self.curr_waypoint_idx = 0 if self.start_direction == 1 else len(self.pose_queue) - 2
        self.get_logger().info(f'First goal index: {self.curr_waypoint_idx}, Total waypoints: {len(self.pose_queue)}')

    def _wait_for_future_no_spin(self, future, timeout_sec: float, label: str) -> bool:
        """Wait for a Future without nested spinning inside callbacks.

        The node is already being spun by the executor in main(), so polling avoids
        re-entering spin() from callback contexts.
        """
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if (time.monotonic() - start) >= timeout_sec:
                self.get_logger().error(f'{label} timed out after {timeout_sec:.1f}s')
                return False
            time.sleep(0.05)

        if not future.done():
            self.get_logger().error(f'{label} did not complete')
            return False

        return True

    def handle_list_waypoints_request(self, req, resp):
        self.get_logger().info('[ENTRY] handle_list_waypoints_request called')
        try:
            if not self.waypoints:
                resp.success = False
                resp.message = "Waypoints have not been loaded."
                return resp
            
            ids = list(self.waypoints.keys())
            if not self.is_clockwise:
                ids = ids[-2::-1] + [ids[-1]]
            lines = []
            for wp_id in ids:
                desc = "unknown"
                if 0 <= wp_id < len(self.pose_queue):
                    desc = self.pose_queue[wp_id][1]
                lines.append(f"{wp_id}:{desc}")

            msg = "Available waypoint IDs (in order of execution): \n" + "\n".join(lines)
            self.get_logger().info(msg)
            resp.success = True
            resp.message = "Waypoints Listed"
            return resp
        finally:
            self.get_logger().info('[EXIT] handle_list_waypoints_request returning')


    def handle_respawn_request(self, req, resp):
        self.get_logger().info('[ENTRY] handle_respawn_request called')
        with self.cv_respawning:
            self.respawning = True
               
        try:
            self.get_logger().info(f'Respawn service start for waypoint id={req.spawn_waypoint_id}')

            if req.spawn_waypoint_id not in self.waypoints.keys():
                resp.success = False
                resp.message = "Id does not exist"
                return resp

            # cancel current goal
            if hasattr(self, "_current_goal_handle") and self._current_goal_handle:
                try:
                    self.get_logger().info('Respawn stage: cancel current nav goal')
                    self._current_goal_handle.cancel_goal_async()
                except Exception:
                    pass

            raw_waypoint = self.waypoints[req.spawn_waypoint_id]
            if not isinstance(raw_waypoint, (list, tuple)) or len(raw_waypoint) != 2:
                resp.success = False
                resp.message = "Waypoint format invalid; expected [lat, lon]"
                return resp

            # self.waypoints stores [latitude, longitude].
            latitude, longitude = raw_waypoint[0], raw_waypoint[1]

            if not (0 <= req.spawn_waypoint_id < len(self.pose_queue)):
                resp.success = False
                resp.message = "Waypoint index out of range for pose_queue"
                return resp

            waypoint_pos = self.pose_queue[req.spawn_waypoint_id][0]
            self.get_logger().info(
                f"Respawn requested: id={req.spawn_waypoint_id}, map=("
                f"{waypoint_pos.pose.position.x:.2f}, {waypoint_pos.pose.position.y:.2f})"
            )

            self.get_logger().info('Respawn stage: teleport in Gazebo')
            if not self.teleport_in_gazebo(latitude, longitude, req.spawn_waypoint_id, timeout_sec=5.0):
                resp.success = False
                resp.message = "Rover could not teleport"
                return resp

            self.get_logger().info('Respawn stage: reset odom stack')
            if not self.reset_odom_stack(req.spawn_waypoint_id, timeout_sec=8.0):
                resp.success = False
                resp.message = "Odometry could not be restarted."
                return resp

            

            self.get_logger().info('Respawn stage: clear Nav2 costmaps (non-blocking)')
            self.clearAllCostmaps(wait_for_result=False)
            self.curr_waypoint_idx = req.spawn_waypoint_id + 1


            resp.success = True
            resp.message = f"Respawning the Rover at {req.spawn_waypoint_id}"
            self.get_logger().info(f'Respawn service success for waypoint id={req.spawn_waypoint_id}')
            return resp
        except Exception as e:
            self.get_logger().error(f'Respawn service exception: {e}')
            resp.success = False
            resp.message = f'Respawn failed: {e}'
            return resp
        finally:
            with self.cv_respawning:    
                self.get_logger().info('[EXIT] handle_respawn_request returning')
                
                self.respawning = False
                self.cv_respawning.notify_all()
            
        
    def reset_odom_stack(self, waypoint_idx, timeout_sec = 2.0):
        if not (0 <= waypoint_idx < len(self.pose_queue)):
            self.get_logger().error(f'Invalid waypoint index for reset: {waypoint_idx}')
            return False

        pose_stamped = self.pose_queue[waypoint_idx][0]
        self.get_logger().info(
            f"Reset pose input: frame={pose_stamped.header.frame_id or 'map'}, "
            f"pos=({pose_stamped.pose.position.x:.2f}, {pose_stamped.pose.position.y:.2f}), "
            f"quat=({pose_stamped.pose.orientation.x:.3f}, {pose_stamped.pose.orientation.y:.3f}, "
            f"{pose_stamped.pose.orientation.z:.3f}, {pose_stamped.pose.orientation.w:.3f})"
        )

        # pose_queue is populated in map frame, so use it directly for global EKF reset.
        map_pose = PoseStamped()
        map_pose.header.frame_id = 'map'
        map_pose.header.stamp = Time().to_msg() 
        map_pose.pose = pose_stamped.pose

        self.get_logger().info(
            f"Reset pose map: pos=({map_pose.pose.position.x:.2f}, {map_pose.pose.position.y:.2f}), "
            f"quat=({map_pose.pose.orientation.x:.3f}, {map_pose.pose.orientation.y:.3f}, "
            f"{map_pose.pose.orientation.z:.3f}, {map_pose.pose.orientation.w:.3f})"
        )


        try:
            odom_pose = self.tf_buffer.transform(map_pose, 'odom', timeout=Duration(seconds=2))
        except Exception as e:
            self.get_logger().error(f'map to odom transform failed: {e}')
            return False


        if not self.restart_cartographer_trajectory(map_pose, timeout_sec=timeout_sec):
            self.get_logger().error('Failed to restart Cartographer trajectory at respawn pose')
            return False

        # Covariance tuned to avoid filter instability right after teleport/reset.
        cov = [0.0] * 36
        cov[0] = 0.15       # x
        cov[7] = 0.15       # y
        cov[14] = 9999.0    # z (not estimated)
        cov[21] = 9999.0    # roll (not estimated)
        cov[28] = 9999.0    # pitch (not estimated)
        cov[35] = 0.20      # yaw

        odom_msg = PoseWithCovarianceStamped()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.pose.pose = odom_pose.pose
        odom_msg.pose.covariance = cov

        def call_set_pose(client, service_name, pose_msg):
            req = SetPose.Request()
            req.pose = pose_msg

            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().error(f'{service_name} not available')
                return False

            future = client.call_async(req)
            if not self._wait_for_future_no_spin(future, timeout_sec, f'{service_name} call'):
                return False

            result = future.result()
            if result is None:
                self.get_logger().error(f'{service_name} returned no response')
                return False

            self.get_logger().info(
                f"{service_name} accepted reset pose in frame '{pose_msg.header.frame_id}'"
            )

            return True

        ok_local = call_set_pose(self.ekf_local_set_pose_client, '/ekf_local/set_pose', odom_msg)
        if not ok_local:
            self.get_logger().info('Unsuccessfully reset local(odom) EKF')
            return False
        self.get_logger().info('Successfully reset local EKF')
        return True

    def restart_cartographer_trajectory(self, map_pose: PoseStamped, timeout_sec: float = 3.0) -> bool:
        """Restart Cartographer trajectory with map_pose as initial pose."""
        previous_trajectory_id = int(self.cartographer_active_trajectory_id)
        if not self.finish_trajectory_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('/finish_trajectory not available')
            return False
        if not self.start_trajectory_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('/start_trajectory not available')
            return False

        finish_req = FinishTrajectory.Request()
        self.get_logger().info(f"Finished Trajectorty: {previous_trajectory_id}")
        finish_req.trajectory_id = previous_trajectory_id
        finish_future = self.finish_trajectory_client.call_async(finish_req)
        if (not self._wait_for_future_no_spin(finish_future, timeout_sec, '/finish_trajectory call')
                or finish_future.result() is None):
            self.get_logger().warn(
                f"Could not finish trajectory {self.cartographer_active_trajectory_id}; "
                'continuing with start_trajectory'
            )

        start_req = StartTrajectory.Request()
        start_req.configuration_directory = str(self.cartographer_config_dir)
        start_req.configuration_basename = str(self.cartographer_config_basename)
        start_req.use_initial_pose = True
        start_req.initial_pose = map_pose.pose
        # Keep building on the same map by anchoring the new trajectory to previous one.
        start_req.relative_to_trajectory_id = previous_trajectory_id

        start_future = self.start_trajectory_client.call_async(start_req)
        if not self._wait_for_future_no_spin(start_future, timeout_sec, '/start_trajectory call'):
            return False

        start_resp = start_future.result()
        if start_resp is None:
            self.get_logger().error('/start_trajectory returned no response')
            return False

        self.cartographer_active_trajectory_id = int(start_resp.trajectory_id)
        self.get_logger().info(
            f"Cartographer started trajectory {self.cartographer_active_trajectory_id} at respawn pose"
        )
        return True

    def latlon_to_gazebo(self, latitude: float, longitude: float):
        # Hardcoded lat/lon -> Gazebo conversion path only (no TF lookup).
        # Assumption: Gazebo (0,0) corresponds to geodetic origin (lat, lon) = (0, 0)
        # in the currently used UTM zone.
        target_utm = utm.from_latlon(
            latitude,
            longitude,
            force_zone_number=self.utm_zone,
            force_zone_letter=self.utm_letter,
        )
        origin_utm = utm.from_latlon(
            0.0,
            0.0,
            force_zone_number=self.utm_zone,
            force_zone_letter=self.utm_letter,
        )

        east = target_utm[0] - origin_utm[0]
        north = target_utm[1] - origin_utm[1]

        # Inverse of gazebo_to_latlon rotation; heading defaults to 0 if unset.
        import math
        heading_deg = float(getattr(self, 'gazebo_heading_deg', 0.0))
        th = math.radians(heading_deg)
        c = math.cos(th)
        s = math.sin(th)
        x = c * east + s * north
        y = -s * east + c * north

        gz_pose = Pose()
        gz_pose.position.x = x
        gz_pose.position.y = y
        gz_pose.position.z = 0.05
        # gz_pose.orientation.z = 0.70710678
        # gz_pose.orientation.w = 0.70710678
        gz_pose.orientation.w = 1.0
        return gz_pose

        
    def teleport_in_gazebo(self, latitude, longitude, idx, timeout_sec=2.0): # helper function for teleporting the rover
        if not self.set_pose_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('/world/default/set_pose service not available')
            return False

        req = SetEntityPose.Request()
        req.entity.name = "timbot"
        req.entity.type = Entity.MODEL # double check this 
        gz_pose = self.latlon_to_gazebo(latitude, longitude)

        # # Keep yaw coherent with the waypoint in map frame when available.
        # if 0 <= idx < len(self.pose_queue):
        #     gz_pose.orientation = self.pose_queue[idx][0].pose.orientation

        req.pose = gz_pose
    
        req.pose.position.z = 0.05 # keep slightly above ground to not spawn inside the ground
        
        self.get_logger().info(
            f"Teleporting model '{req.entity.name}' in Gazebo to ("
            f"{req.pose.position.x:.2f}, {req.pose.position.y:.2f}, {req.pose.position.z:.2f})"
        )

        future = self.set_pose_client.call_async(req)
        if not self._wait_for_future_no_spin(future, timeout_sec, '/world/default/set_pose call'):
            return False
        
        result = future.result()
        if result is None or not result.success:
            status = '' if result is None else f" status_message='{result.status_message}'"
            self.get_logger().error(f'Gazebo set_pose failed.{status}')
            return False
        if idx == 0:
            self.laps -= 1
        return True
    
    def clearAllCostmaps(self, wait_for_result: bool = False) -> None:
        """Clear all costmaps.

        When wait_for_result is False, this only dispatches clear requests and returns
        immediately so respawn flow is never blocked by costmap service timeouts.
        """
        self.clearLocalCostmap(timeout_sec=2.0, wait_for_result=wait_for_result)
        self.clearGlobalCostmap(timeout_sec=2.0, wait_for_result=wait_for_result)
        return

    def clearLocalCostmap(self, timeout_sec: float = 2.0, wait_for_result: bool = True) -> None:
        """Clear local costmap."""
        # In fire-and-forget mode, avoid blocking on service discovery.
        service_wait_timeout = timeout_sec if wait_for_result else 0.1
        if not self.clear_local_costmap_client.wait_for_service(timeout_sec=service_wait_timeout):
            self.get_logger().warn('Clear local costmap service unavailable; skipping clear')
            return

        req = ClearEntireCostmap.Request()
        future = self.clear_local_costmap_client.call_async(req)
        if not wait_for_result:
            self.get_logger().info('Dispatched local costmap clear request (non-blocking)')
            return

        if not self._wait_for_future_no_spin(future, timeout_sec, 'Clear local costmap call'):
            self.get_logger().warn('Clear local costmap call timed out; continuing')
            return

        result = future.result()
        if result is None:
            self.get_logger().error('Clear local costmap request failed!')

        return

    def clearGlobalCostmap(self, timeout_sec: float = 2.0, wait_for_result: bool = True) -> None:
        """Clear global costmap."""
        # In fire-and-forget mode, avoid blocking on service discovery.
        service_wait_timeout = timeout_sec if wait_for_result else 0.1
        if not self.clear_global_costmap_client.wait_for_service(timeout_sec=service_wait_timeout):
            self.get_logger().warn('Clear global costmap service unavailable; skipping clear')
            return

        req = ClearEntireCostmap.Request()
        future = self.clear_global_costmap_client.call_async(req)
        if not wait_for_result:
            self.get_logger().info('Dispatched global costmap clear request (non-blocking)')
            return

        if not self._wait_for_future_no_spin(future, timeout_sec, 'Clear global costmap call'):
            self.get_logger().warn('Clear global costmap call timed out; continuing')
            return

        result = future.result()
        if result is None:
            self.get_logger().error('Clear global costmap request failed!')

        return


    def populate_waypoint_dict(self):
        """Load waypoints from JSON file and convert all to map-frame poses."""
        if not self.waypoints_file or not os.path.exists(self.waypoints_file):
            self.get_logger().warn(f'No valid waypoints file configured. Waypoint auto-navigation disabled.')
            self.pose_queue = []
            return
        
        try:
            with open(self.waypoints_file, 'r') as f:
                waypoint_data = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints file: {e}')
            self.pose_queue = []
            return
        # self.waypoints = {wp["id"]:[wp["latitude"], wp["longitude"]] for wp in waypoint_data["waypoints"]}
        self.start_direction = 1 if waypoint_data.get("start_direction", "north") == "north" else -1
        self.laps = waypoint_data.get("laps", 1)

        self.get_logger().info(f'Start direction: {"East" if self.start_direction == 1 else "West"}')
        self.get_logger().info(f'Laps: {self.laps}')

        # Wait for UTM transform (indicates GPS is ready)
        self.waited_for_transform = self.wait_for_utm_transform()

        if not self.waited_for_transform:
            self.get_logger().warn('Waiting for transform from /map to /utm timed out!')
            return

        # Get initial GPS position and convert to UTM
        gps_info = self.wait_for_gps_fix()
        if not gps_info:
            self.get_logger().error('Failed to get GPS fix')
            return

        start_utm = utm.from_latlon(gps_info.latitude, gps_info.longitude)
        self.start_easting = start_utm[0]
        self.start_northing = start_utm[1]
        self.utm_zone = start_utm[2]
        self.utm_letter = start_utm[3]
        self.get_logger().info(f'Start UTM: ({self.start_easting:.2f}, {self.start_northing:.2f}) Zone {self.utm_zone}{self.utm_letter}')

        frame = waypoint_data["waypoints"][0].get("frame_id", "map")
        self.is_clockwise = waypoint_data["clockwise"]

        # Convert each waypoint lat/lon to UTM, then transform to map frame
        midpoint_utms = []
        for i, wp in enumerate(waypoint_data["waypoints"]):
            self.waypoints[i] = [wp["latitude"], wp["longitude"]]
            utm_coords = utm.from_latlon(wp["latitude"], wp["longitude"], force_zone_number=self.utm_zone, force_zone_letter=self.utm_letter)
            midpoint_utms.append({
                'easting': utm_coords[0],
                'northing': utm_coords[1],
                'description': wp.get("description", f"Waypoint {wp['id']}")
            })
            self.get_logger().info(f'Midpoint {wp.get("description", wp["id"])}: UTM ({utm_coords[0]:.2f}, {utm_coords[1]:.2f})')

        # Build waypoint list with corners if requested
        if waypoint_data.get("add_corners", False):
            waypoint_utms = self.add_corners_utm(midpoint_utms)
        else:
            waypoint_utms = midpoint_utms

        if not self.is_clockwise:
            self.get_logger().warn("ATTENTION: Waypoints now in counter-clockwise order!!! Rover Respawn will still use the idx from the .json file!")
            waypoint_utms = waypoint_utms[-2::-1] + [waypoint_utms[-1]] # assumes that you put the start location as the last point

        # Convert all UTM coordinates to map-frame poses
        for i, wp_utm in enumerate(waypoint_utms):
            pose = self.utm_to_map_pose(wp_utm['easting'], wp_utm['northing'], frame)
            if pose:
                self.pose_queue.append((pose, wp_utm['description']))
                self.get_logger().info(f'pose frame: {pose.header.frame_id}')
                self.get_logger().info(f'Waypoint {i}: {wp_utm["description"]} -> ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')
            else:
                self.get_logger().warn(f'Failed to convert waypoint {i} to map frame')
        
        self.get_logger().info(f'Loaded {len(self.pose_queue)} waypoints into queue')

    def add_corners_utm(self, midpoints):
        """
        Add corner waypoints between midpoints using UTM coordinates.
        Expects 4 midpoints: East, South, West, North (in order)
        Robot starts facing east, so we go clockwise starting with SE corner.
        Returns: 8 waypoints (4 corners + 4 midpoints)
        """
        if len(midpoints) != 4:
            self.get_logger().warn(f'Expected 4 midpoints, got {len(midpoints)}. Skipping corners.')
            return midpoints

        result = []

        # lop to compute the midpoints:
        for i in range(4):
            curr = midpoints[i]
            next = midpoints[(i + 1) % 4]

            # Compute corner as a combination of current and next midpoints based on the direction of the turn
            corner_easting = curr['easting'] if self.start_direction == 1 else next['easting']
            corner_northing = next['northing'] if self.start_direction == 1 else curr['northing']
            result.append(curr)
            result.append({
                'easting': corner_easting,
                'northing': corner_northing,
                'description': f'Corner between {curr["description"]} and {next["description"]}'
            })
        
        # move the last point to the start
        result.insert(0, result.pop())
        return result

    def utm_to_map_pose(self, easting, northing, frame):
        """Convert UTM coordinates to a pose in the specified frame."""
        utm_pose = PoseStamped()
        utm_pose.header.frame_id = 'utm'
        utm_pose.header.stamp = self.get_clock().now().to_msg()
        utm_pose.pose.position.x = easting
        utm_pose.pose.position.y = northing
        utm_pose.pose.orientation.w = 1.0

        try:
            return self.tf_buffer.transform(utm_pose, frame, timeout=Duration(seconds=2))
        except Exception as e:
            self.get_logger().error(f'UTM to {frame} transform failed: {e}')
            return None
        
    def wait_for_gps_fix(self, timeout=10.0):
        """Wait for a GPS fix message."""
        gps_msg = None

        def callback(msg):
            nonlocal gps_msg
            gps_msg = msg

        sub = self.create_subscription(NavSatFix, '/gps/filtered', callback, 10)

        start = time.time()
        while gps_msg is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.destroy_subscription(sub)
        return gps_msg

    def wait_for_utm_transform(self):
        """Wait for map->utm transform to become available (GPS ready)."""
        self.get_logger().info('Waiting for map->utm transform...')

        start_time = time.time()
        while (time.time() - start_time) < self.max_time_for_transform:
            try:
                self.tf_buffer.lookup_transform('map', 'utm', Time())
                self.get_logger().info(f'Transform found after {time.time() - start_time:.1f}s')
                return True
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)

        return False

    def get_next_waypoint(self):
        """Get the next waypoint pose and description, update index."""
        pose, description = self.pose_queue[self.curr_waypoint_idx]
        self.get_logger().info(f'Next Goal: {description} (idx {self.curr_waypoint_idx})')

        # Determine if we should ignore lidar (e.g., in certain segments)
        if self.curr_waypoint_idx == 3 and self.start_direction == 1:
            self.ignore_lidar = True
        elif self.curr_waypoint_idx == 2 and self.start_direction == -1:
            self.ignore_lidar = True
        else:
            self.ignore_lidar = False

        # Publish ignore lidar state
        msg = Bool()
        msg.data = self.ignore_lidar
        for _ in range(10):
            self.waypoint_pub.publish(msg)

        # Update index
        self.curr_waypoint_idx += self.start_direction
        if self.curr_waypoint_idx < 0 and self.current_lap < self.laps:
            self.current_lap += 1
            self.curr_waypoint_idx = len(self.pose_queue) - 1
        elif self.curr_waypoint_idx >= len(self.pose_queue) and self.current_lap < self.laps:
            self.current_lap += 1
            self.curr_waypoint_idx = 0

        return pose, description

    def get_pose_from_gps(self, longitude, latitude, frame):
        """Convert GPS coordinates to pose in specified frame."""
        utm_coords = utm.from_latlon(latitude, longitude)

        utm_pose = PoseStamped()
        utm_pose.header.frame_id = 'utm'
        utm_pose.header.stamp = self.get_clock().now().to_msg()
        utm_pose.pose.position.x = utm_coords[0]
        utm_pose.pose.position.y = utm_coords[1]
        utm_pose.pose.orientation.w = 1.0

        try:
            p_in_frame = self.tf_buffer.transform(utm_pose, frame, timeout=Duration(seconds=1))
            return p_in_frame
        except Exception as e:
            self.get_logger().error(f'Transform failed: {e}')
            return None

    def send_goal_to_nav2(self, pose, description):
        """Send a pre-converted pose to Nav2 and wait for result."""
        action_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose',
            callback_group=self.action_cb_group
        )

        if not action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available!')
            return "RETRY_SAME"

        # Update timestamp on pose
        pose.header.stamp = self.get_clock().now().to_msg()

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f'Sending goal: {description} ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')

        self.goal_done_event.clear()
        self.goal_result = None
        self._current_goal_handle = None

        future = action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

        # Wait for goal acceptance
        start_time = time.time()
        while self._current_goal_handle is None and (time.time() - start_time) < 10.0:
            if not rclpy.ok():
                self.get_logger().info('ROS shutting down, aborting goal acceptance wait.')
                return "RETRY_SAME"
            time.sleep(0.1)

        if self._current_goal_handle is None or not self._current_goal_handle.accepted:
            self.get_logger().error('Goal rejected or timed out')
            return "RETRY_SAME"

        self.get_logger().info('Goal accepted, waiting for result...')

        # Wait for result, respawn interrupt, or ramp interrupt using event-based waiting
        while not self.goal_done_event.is_set():
            if not rclpy.ok():
                self.get_logger().info('ROS shutting down, aborting goal execution wait.')
                return "RETRY_SAME"
                
            # Check periodically with short timeout
            if self.goal_done_event.wait(timeout=0.5):
                break
            
            # handle respawn interrupt
            with self.cv_respawning:
                if self.respawning:
                    self.get_logger().info('Navigatio interrupted for respawning')
                    if self._current_goal_handle:
                        self._current_goal_handle.cancel_goal_async()
                    self.cv_respawning.wait_for(lambda: not self.respawning)
                    self.get_logger().info('Rover Respawned. Resumine navigation to the next new waypoint.')

                    # return False so the rety loop sends the new? waypoint again
                    return "REPLAN"
            
            # Handle ramp interrupt
            with self.cv_ramp_naving:
                if self.ramp_naving:
                    self.get_logger().info('Navigation interrupted for ramp crossing')
                    if self._current_goal_handle:
                        self._current_goal_handle.cancel_goal_async()
                    self.cv_ramp_naving.wait_for(lambda: not self.ramp_naving)
                    self.get_logger().info('Ramp crossed. Resuming navigation to the current waypoint.')
                    
                    # Return False so the retry loop sends the SAME waypoint again
                    return "RETRY_SAME"

        # We only reach this point if the goal has finished (success or failure)
        if self.goal_result and self.goal_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Successfully reached: {description}")
        else:
            self.get_logger().warn(f"Goal aborted or failed during execution: {description}. Moving to next waypoint.")
            
        # Return True so the retry loop breaks and we fetch the next waypoint
        return "SUCCESS"

    def _goal_response_callback(self, future):
        """Callback when goal response is received."""
        self._current_goal_handle = future.result()
        if self._current_goal_handle.accepted:
            result_future = self._current_goal_handle.get_result_async()
            result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """Callback when goal result is received."""
        self.goal_result = future.result()
        self.goal_done_event.set()

    def goal_pose_callback(self, msg):
        """Stop autonomous waypoint loop when RViz sends a manual goal."""
        if self.stop_waypoint_loop:
            return

        self.stop_waypoint_loop = True
        self.get_logger().warn(
            'RViz /goal_pose received. Stopping load_waypoints loop; RViz now controls navigation.'
        )

        # Cancel any in-flight autonomous goal so manual goal can take over immediately.
        if hasattr(self, '_current_goal_handle') and self._current_goal_handle:
            try:
                self._current_goal_handle.cancel_goal_async()
            except Exception:
                pass

    def navigate_waypoints(self):
        """Main navigation loop - iterates through all waypoints in pose_queue."""
        if not self.pose_queue:
            self.get_logger().error('No waypoints loaded!')
            return

        # self.clear_costmaps() Try without this
        while rclpy.ok():
            if self.stop_waypoint_loop:
                self.get_logger().info('Autonomous waypoint loop stopped by manual RViz goal.')
                break

            # if self.current_lap >= self.laps:
            #     self.get_logger().info('All laps completed!')
            #     break

            if self.curr_waypoint_idx < 0 or self.curr_waypoint_idx >= len(self.pose_queue):
                self.get_logger().info('All waypoints visited!')
                break

            # Fetch the target pose once
            with self.cv_respawning:
                self.cv_respawning.wait_for(lambda: not self.respawning)
            pose, description = self.get_next_waypoint()
            
            # Keep trying this specific pose until Nav2 accepts and completes it
            success = "RETRY_SAME"
            while rclpy.ok() and success != "SUCCESS":
                if self.stop_waypoint_loop:
                    self.get_logger().info('Autonomous waypoint retries stopped by manual RViz goal.')
                    return

                success = self.send_goal_to_nav2(pose, description)
                
                if success == "RETRY_SAME":
                    if self.stop_waypoint_loop:
                        self.get_logger().info('Stopping retry because manual RViz goal is active.')
                        return
                    self.get_logger().warn(f"Goal '{description}' was rejected or failed. Retrying in 3 seconds...")
                    time.sleep(3.0)  # Safe to use time.sleep here because it's in a daemon thread!
                elif success == "REPLAN":
                    if self.stop_waypoint_loop:
                        self.get_logger().info('Stopping replan because manual RViz goal is active.')
                        return
                    pose, description = self.get_next_waypoint()
                    self.get_logger().warn(f"Rover Respawned. New Goal '{description}' is now set. Retrying in 3 seconds...")
                    time.sleep(3.0)

    def ramp_naving_callback(self, msg):
        """Handle ramp navigation state changes."""
        with self.cv_ramp_naving:
            self.ramp_naving = msg.data
            if not self.ramp_naving:
                self.cv_ramp_naving.notify_all()


def main(args=None):
    rclpy.init(args=args)

    node = NavigateWaypoints()

    # Use MultiThreadedExecutor to handle callbacks from background thread.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Run navigation in separate thread (Add daemon=True)
    nav_thread = th.Thread(target=node.navigate_waypoints, name='navigate_waypoints', daemon=True)
    nav_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nav_thread.join(timeout=5.0)
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()