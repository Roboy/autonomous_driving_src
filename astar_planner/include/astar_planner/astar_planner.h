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
#include <unordered_map>

#include "astar_planner/costmap.h"
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
                        costmap_2d::Costmap2DROS *costmap_ros) override;

        void initialize(std::string name, Costmap *costmap);

        /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose
       * @param goal The goal pose
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan) override;

        bool makePlan(const Position &start, const Position &goal, std::vector<Position> &path);

        void getPath(const std::unordered_map<Position, Position> &parents,
                     const Position &goal_pos,
                     std::vector<Position> &path) const;


        std::vector<PosWithDist> getNeighbors(const Position &pos) const;

        double distEstimate(const Position &pose1, const Position &pose2) const;

        Cell getCell(const Position &pos) const;

        void loadParameters();

        bool validateParameters() const;

        bool checkBounds(const Position &pos) const;

        PosWithDist goStraight(const Position &pos) const;

        PosWithDist turnLeft(const Position &pos, double angle) const;

        PosWithDist turnRight(const Position &pos, double angle) const;



    private:
        std::string name_;
        Costmap *costmap_;
        std::string global_frame_;

        double step_size_;
        double turning_radius_;
    };


}

#endif //ASTAR_PLANNER_ASTAR_H
