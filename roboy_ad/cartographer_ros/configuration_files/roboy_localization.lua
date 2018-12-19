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

include "utum_indoors.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
	max_submaps_to_keep = 5,
}


--POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

--POSE_GRAPH.constraint_builder.min_score = 0.65

POSE_GRAPH.constraint_builder.sampling_ratio = 0.05

--POSE_GRAPH.max_num_final_iterations = 1

POSE_GRAPH.constraint_builder.max_constraint_distance = 25
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 25
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(180.0)

POSE_GRAPH.optimize_every_n_nodes = 2

return options




