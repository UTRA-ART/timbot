-- Simplified Cartographer config for isolated odometry / simple_nav testing.
-- Differences from description/config/cartographer.lua:
--   - num_point_clouds = 0  (no ZED obstacle or lane clouds)
--   - num_laser_scans  = 1  (one RPLidar on /scan_lower)
-- Everything else is inherited from the tuned base config.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  publish_to_tf = true,
  provide_odom_frame = false,
  publish_tracked_pose = true,
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
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 400
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 400
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(5.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 45
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.constraint_builder.sampling_ratio = 0.4
POSE_GRAPH.constraint_builder.min_score = 0.85
POSE_GRAPH.global_constraint_search_after_n_seconds = 30
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e6
POSE_GRAPH.optimization_problem.odometry_translation_weight = 2e5
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e0

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100

return options
