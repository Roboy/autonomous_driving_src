//
// Created by alex on 13.02.19.
//

#ifndef ASTAR_PLANNER_ASTAR_H
#define ASTAR_PLANNER_ASTAR_H


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>
// Abstract global planner from move_base
#include <nav_core/base_global_planner.h>
#include <vector>

#include "astar_planner/utils.h"

namespace astar_planner {

    class AStarPlanner : public nav_core::BaseGlobalPlanner {
    public:
        AStarPlanner();
        ~AStarPlanner();
        /**
          * @brief  Initialization function for the RRTPLaner object
          * @param  name The name of this planner
          * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
          */
        void initialize(std::string name,
                        costmap_2d::Costmap2DROS *costmap_ros);

        /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose
       * @param goal The goal pose
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector <geometry_msgs::PoseStamped> &plan);
        std::vector<PoseWithDist> getNeighbors(const geometry_msgs::PoseStamped &pose);

    private:
        std::string name_;
        costmap_2d::Costmap2DROS *costmap_ros_;

    };


}

#endif //ASTAR_PLANNER_ASTAR_H
