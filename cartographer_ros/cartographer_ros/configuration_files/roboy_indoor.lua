include "map_builder.lua"
include "trajectory_builder.lua"
include "rickshaw_local.lua"

-- -- LOCAL -- --
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 3
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 70

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 5 --20

--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.01
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight= 0.01 

-- -- IMU -- --
TRAJECTORY_BUILDER_2D.use_imu_data = true
POSE_GRAPH.optimization_problem.acceleration_weight = 1e5
POSE_GRAPH.optimization_problem.rotation_weight = 1e2


-- -- GLOBAL -- --
POSE_GRAPH.optimize_every_n_nodes = 100
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 28

POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 28
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = true

POSE_GRAPH.constraint_builder.max_constraint_distance = 15
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 15
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(60.0)

--POSE_GRAPH.constraint_builder.sampling_ratio = 0.05

--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 6

POSE_GRAPH.constraint_builder.min_score = 0.60

--POSE_GRAPH.constraint_builder.ceres_scan_matcher

POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e3
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e4
--POSE_GRAPH.matcher_translation_weight = 1e1
--POSE_GRAPH.matcher_rotation_weight = 1e3
--POSE_GRAPH.optimization_problem.*_weight
--POSE_GRAPH.optimization_problem.ceres_solver_options

--POSE_GRAPH.max_num_final_iterations = 10

return options
