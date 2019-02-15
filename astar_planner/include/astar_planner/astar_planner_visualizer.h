//
// Created by alex on 13.02.19.
//

#ifndef ASTAR_PLANNER_ASTAR_PLANNER_VISUALIZER_H
#define ASTAR_PLANNER_ASTAR_PLANNER_VISUALIZER_H


#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <astar_planner/astar_planner.h>

namespace astar_planner {
    class AStarPlannerVisualizer {
    private:
        ros::NodeHandle handle_;
        AStarPlanner planner_;
        ros::Publisher plan_publisher_;
        costmap_2d::Costmap2DROS *costmap_;

    public:
        AStarPlannerVisualizer();
        ~AStarPlannerVisualizer();
        void initialize();
        void updateInitialPosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        void updateGoal(const geometry_msgs::PoseStamped::ConstPtr& msg);

    };
}

#endif //ASTAR_PLANNER_ASTAR_PLANNER_VISUALIZER_H
