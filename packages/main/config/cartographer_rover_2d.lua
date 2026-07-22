-- Cartographer 2D configuration for the RoboMarvel rover.
-- Sensor topology:
--   LDROBOT LD19  -> /scan (sensor_msgs/LaserScan, frame: lidar)
--   RTAB-Map ICP  -> /icp/odom_raw
--   Odom sanitizer -> /icp/odom and TF odom -> base_link
--   Cartographer  -> TF map -> odom and /map via occupancy-grid node
--
-- Cartographer is deliberately configured not to create its own odom frame,
-- preventing a competing odom -> base_link transform publisher.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",

  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,

  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.25,
  submap_publish_period_sec = 0.30,
  pose_publish_period_sec = 0.020,
  trajectory_publish_period_sec = 0.050,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2

-- LD19 is a planar lidar. The rover's IMU message currently does not contain
-- an orientation estimate, so forcing IMU use would degrade or stop mapping.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025

-- Correlative matching increases robustness to wheel slip and imperfect ICP.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 2.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.06
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(2.5)

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.025

-- Keep mapping responsive on Raspberry Pi while retaining loop closures.
POSE_GRAPH.optimize_every_n_nodes = 240
POSE_GRAPH.constraint_builder.sampling_ratio = 0.04
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 2
POSE_GRAPH.log_residual_histograms = false
POSE_GRAPH.constraint_builder.log_matches = false
POSE_GRAPH.max_num_final_iterations = 20

return options
