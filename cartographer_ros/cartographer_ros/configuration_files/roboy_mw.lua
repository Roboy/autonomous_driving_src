include "map_builder.lua"
include "trajectory_builder.lua"
include "rickshaw_local.lua"

-- -- IMU -- --
TRAJECTORY_BUILDER_2D.use_imu_data = true

--    huber_scale = 1e1,
--    acceleration_weight = 1e3,
--    rotation_weight = 3e5,
--POSE_GRAPH.optimization_problem.huber_scale = 1e-1
POSE_GRAPH.optimization_problem.acceleration_weight = 1e5
POSE_GRAPH.optimization_problem.rotation_weight = 5e2


-- -- GLOBAL -- --
POSE_GRAPH.optimize_every_n_nodes = 100
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 28

POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 28
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = true

POSE_GRAPH.constraint_builder.max_constraint_distance = 25
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 35
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(180.0)

--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 6

--POSE_GRAPH.constraint_builder.sampling_ratio = 0.7
POSE_GRAPH.constraint_builder.min_score = 0.58 -- 0.53 -- 0.59

--POSE_GRAPH.constraint_builder.ceres_scan_matcher

--    loop_closure_translation_weight = 1.1e4
--    loop_closure_rotation_weight = 1e5
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 9e3 --9e3
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 8e4

--  matcher_translation_weight = 5e2,
--  matcher_rotation_weight = 1.6e3,
POSE_GRAPH.matcher_translation_weight = 3e2 --8e2
--POSE_GRAPH.matcher_rotation_weight = 1e3 --3e3

--POSE_GRAPH.max_num_final_iterations = 1

return options
