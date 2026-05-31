-- Simplified Cartographer config for isolated odometry / simple_nav testing.
-- Differences from description/config/cartographer.lua:
--   - num_laser_scans  = 1  (one RPLidar on /scan_lower)
--   - num_point_clouds = 2  (ZED obstacle + lane clouds)
--   - publish_tracked_pose = false (not used in simple_nav loop)
--
-- Tuning philosophy: ekf_local is the sole pose source. Cartographer is a
-- pure map painter — it stamps lidar returns at exactly the pose ekf_local
-- reports, with no scan-based corrections at all.
--
--   ceres_scan_matcher weights ≈ 0: Ceres has nothing to optimise, so it
--     returns the odometry prediction unchanged every scan.
--
--   optimize_every_n_nodes = 0: global pose graph / loop closure is fully
--     disabled. No jumps, no global corrections, ever.
--
--   odometry weights kept high as a safety net in case the global optimizer
--     is re-enabled for future testing.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",  -- imu_link requires use_imu_data=true; we use base_link since IMU is already fused in ekf_local
  published_frame = "odom",
  odom_frame = "odom",
  publish_to_tf = true,
  provide_odom_frame = false,
  publish_tracked_pose = false,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 2,
  lookup_transform_timeout_sec = 1.,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.05,  -- 20 Hz; was 200 Hz (5e-3) which amplified any micro-jitter into the TF
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 0.5,
  imu_sampling_ratio = 0.5,  -- don't consume IMU messages at all; use_imu_data=false makes this a no-op but belt-and-suspenders
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- IMU already fused in ekf_local; using it here double-counts and adds raw IMU jitter to map->odom TF
-- Translation pinned to ekf_local (translation is good, don't touch it).
-- Rotation also pinned: scan-based orientation correction made things worse,
-- so heading is locked to odom as well until a better solution is found.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1e9
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1e9
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1e-9

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(5.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.

POSE_GRAPH.optimization_problem.huber_scale = 1e2
-- Disable global SLAM entirely: no loop closure, no submap matching,
-- no pose graph optimization runs. Zero jumps.
POSE_GRAPH.optimize_every_n_nodes = 0

-- Kept high as a safety net if you re-enable the optimizer later.
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e9
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e9
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e2
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e0

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 10  -- was 100; smaller submaps complete faster so the global map accumulates visibly

return options
