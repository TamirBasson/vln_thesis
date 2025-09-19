-- Include configuration files for map building and trajectory building
include "map_builder.lua"
include "trajectory_builder.lua"

-- Cartographer options and parameters
options = {
  map_builder = MAP_BUILDER, -- Specifies the map builder to use (2D or 3D mapping)
  trajectory_builder = TRAJECTORY_BUILDER, -- Specifies the trajectory builder (handles sensor data integration)
  map_frame = "map", -- The global frame for the map
  tracking_frame = "base_footprint", -- The frame used for tracking the robot's pose
  published_frame = "base_footprint", -- The frame to which Cartographer publishes poses
  odom_frame = "odom", -- The frame for odometry data (usually linked to base_footprint)

  provide_odom_frame = true, -- Allow Cartographer to publish the odometry frame (odom to base_footprint)
  use_odometry = false, -- Do not use external odometry data
  use_pose_extrapolator = false, -- Use a pose extrapolator for smooth pose estimates
  use_nav_sat = false, -- Do not use GPS data for navigation
  use_landmarks = false, -- Do not use landmarks for localization
  publish_to_tf = true,

  publish_frame_projected_to_2d = true, -- Project the published frame onto a 2D plane
  num_laser_scans = 1, -- Use one 2D LiDAR scan
  num_multi_echo_laser_scans = 0, -- No multi-echo LiDAR scans
  num_subdivisions_per_laser_scan = 1, -- Process one laser scan at a time
  num_point_clouds = 0, -- No 3D point clouds
  lookup_transform_timeout_sec = 0.2, -- Timeout duration for TF lookups (in seconds)
  submap_publish_period_sec = 0.3, -- Frequency of submap publication (in seconds)
  pose_publish_period_sec = 5e-3, -- Frequency of pose publication (in seconds)
  trajectory_publish_period_sec = 30e-3, -- Frequency of trajectory publication (in seconds)
  rangefinder_sampling_ratio = 1.0, -- Use all rangefinder data (LiDAR scans)
  odometry_sampling_ratio = 1.0, -- Use all odometry data
  fixed_frame_pose_sampling_ratio = 1.0, -- Use all fixed-frame pose data
  imu_sampling_ratio = 1.0, -- Use all IMU data
  landmarks_sampling_ratio = 1.0, -- Use all landmark data
}

-- Use a 2D trajectory builder
MAP_BUILDER.use_trajectory_builder_2d = true

-- Configure 2D trajectory builder
TRAJECTORY_BUILDER_2D.min_range = 0.5 -- Minimum range for valid LiDAR data (in meters)
TRAJECTORY_BUILDER_2D.max_range = 10.0 -- Maximum range for valid LiDAR data (in meters)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 10.0 -- Length to assign to missing rays (in meters)
TRAJECTORY_BUILDER_2D.use_imu_data = true -- Enable IMU for pose estimation
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true -- Use scan matching for better alignment

-- Parameters for real-time correlative scan matching
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2 -- Search window for linear adjustments (in meters)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 3e4 -- Weight for translation deltas
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 3e4 -- Weight for rotation deltas

-- Motion filtering settings
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.05 -- Maximum time interval for motion filtering (in seconds)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05 -- Maximum distance for motion filtering (in meters)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1) -- Maximum angle change for motion filtering (in radians)

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 -- Number of scans to accumulate before processing

-- Pose graph optimization settings
POSE_GRAPH.constraint_builder.min_score = 0.65 -- Minimum score for constraint building
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.8 -- Minimum score for global localization
POSE_GRAPH.optimization_problem.huber_scale = 1e1 -- Robustness parameter for optimization
POSE_GRAPH.optimize_every_n_nodes = 20 -- Optimize the pose graph after this many nodes
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 9.8 -- Smooth gravity estimation for IMU
-- Return the options to Cartographer
return options