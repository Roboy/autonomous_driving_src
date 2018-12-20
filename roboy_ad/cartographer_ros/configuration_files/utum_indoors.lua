-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  --publish_frame_projected_to_2d = false,
  --use_pose_extrapolator = true,
  use_odometry = false,
  use_laser_scan = true,
  use_multi_echo_laser_scan = false,
  --use_nav_sat = false,
  --use_landmarks = false,
  --num_laser_scans = 1,
  --num_multi_echo_laser_scans = 0,
  --num_subdivisions_per_laser_scan = 10,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  --rangefinder_sampling_ratio = 1.,
  --odometry_sampling_ratio = 1.,
  --fixed_frame_pose_sampling_ratio = 1.,
  --imu_sampling_ratio = 1.,
  --landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 3

-- -- INPUT DATA -- --
TRAJECTORY_BUILDER_2D.min_range = 1
TRAJECTORY_BUILDER_2D.max_range = 25

--TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 20

TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 3
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 70

TRAJECTORY_BUILDER_2D.use_imu_data = false


-- -- LOCAL -- --
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 3
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1e-7
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1e-8

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(135.0)
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.01
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight= 0.01 

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 25
--TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1080

-- TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = 


-- -- GLOBAL -- --
--POSE_GRAPH.optimize_every_n_nodes = 25

--POSE_GRAPH.constraint_builder.max_constraint_distance = 10
--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 10
--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(160.0)

--POSE_GRAPH.constraint_builder.sampling_ratio = 0.05

--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 6

--POSE_GRAPH.constraint_builder.min_score = 0.6

--POSE_GRAPH.constraint_builder.ceres_scan_matcher

--POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e-8
--POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e-8
--POSE_GRAPH.matcher_translation_weight = 1e-8
--POSE_GRAPH.matcher_rotation_weight = 1e-8
--POSE_GRAPH.optimization_problem.*_weight
--POSE_GRAPH.optimization_problem.ceres_solver_options

--POSE_GRAPH.max_num_final_iterations = 10

return options


