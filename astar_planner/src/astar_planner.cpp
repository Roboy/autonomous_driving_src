//
// Created by alex on 13.02.19.
//

#include <pluginlib/class_list_macros.h>

#include <ros/console.h>
#include <queue>

#include <astar_planner/astar_planner.h>
#include <astar_planner/utils.h>

PLUGINLIB_EXPORT_CLASS(astar_planner::AStarPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace astar_planner {

    AStarPlanner::AStarPlanner(): name_(""), costmap_ros_(NULL) {}

    AStarPlanner::~AStarPlanner() {}

    void AStarPlanner::initialize(std::string name,
                                costmap_2d::Costmap2DROS *costmap_ros) {
        name_ = name;
        costmap_ros_ = costmap_ros;
        ROS_INFO("AStarPlanner initialized.");
    };

    bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                              const geometry_msgs::PoseStamped &goal,
                              std::vector <geometry_msgs::PoseStamped> &plan) {
        double start_x = start.pose.position.x;
        double start_y = start.pose.position.y;

        double goal_x = goal.pose.position.x;
        double goal_y = goal.pose.position.y;

        int num_points = 2;
        ros::Time plan_time = ros::Time::now();

        for(int i=0; i <= num_points; i++) {
            double x = start_x + (goal_x - start_x) / num_points * i;
            double y = start_y + (goal_y - start_y) / num_points * i;
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            plan.push_back(pose);
        }

        return true;
    }

    bool AStarPlanner::makeNewPlan(const geometry_msgs::PoseStamped &start,
                                   const geometry_msgs::PoseStamped &goal,
                                   std::vector <geometry_msgs::PoseStamped> &plan) {
        priority_queue<PoseWithDist> candidatePoses;
        while (!candidatePoses.empty()) {
            PoseWithDist cand = candidatePoses.top();
            candidatePoses.pop();
            vector<PoseWithDist> neighbors = getNeighbors(cand.pose);
            for (vector<PoseWithDist>::iterator it = neighbors.begin(); it != neighbors.end(); it++) {

            }
        }

    }

    vector<PoseWithDist> AStarPlanner::getNeighbors(const geometry_msgs::PoseStamped &pose) {

    }
}
