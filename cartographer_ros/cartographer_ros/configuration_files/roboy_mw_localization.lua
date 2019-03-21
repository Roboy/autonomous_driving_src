include "roboy_mw.lua"

TRAJECTORY_BUILDER.pure_localization = true

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false

-- -- GLOBAL -- --
-- std:
--     global_constraint_search_after_n_seconds = 10.,
--     global_localization_min_score = 0.6
--     global_sampling_ratio = 0.003,

POSE_GRAPH.global_constraint_search_after_n_seconds =  5
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
--POSE_GRAPH.global_sampling_ratio = 0.005

-- -- LOCAL -- -- 
--POSE_GRAPH.constraint_builder.sampling_ratio = 0.6
POSE_GRAPH.constraint_builder.min_score = 0.45
POSE_GRAPH.constraint_builder.max_constraint_distance = 15
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 20
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(15.0)

-- -- OPTIMIZATION -- --
POSE_GRAPH.optimize_every_n_nodes = 10
--POSE_GRAPH.max_num_final_iterations = 1

return options
