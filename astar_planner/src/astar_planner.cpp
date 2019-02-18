//
// Created by alex on 13.02.19.
//

#include <pluginlib/class_list_macros.h>

#include <ros/console.h>
#include <queue>
#include <unordered_map>

#include <astar_planner/astar_planner.h>
#include <astar_planner/utils.h>

PLUGINLIB_EXPORT_CLASS(astar_planner::AStarPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace astar_planner {

    AStarPlanner::AStarPlanner(): name_(""), costmap_ros_(nullptr) {}

    AStarPlanner::~AStarPlanner() {}

    void AStarPlanner::initialize(std::string name,
                                costmap_2d::Costmap2DROS *costmap_ros) {
        name_ = name;
        costmap_ros_ = costmap_ros;
        ROS_INFO("AStarPlanner initialized.");
    };

    void AStarPlanner::initialize(std::string name, astar_planner::Costmap *costmap) {
        name_ = name;
        costmap_ = costmap;
    }

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
        auto cell_start = getCell(start);
        auto cell_goal = getCell(goal);
        set<PoseWithDist> candidatePoses;
        unordered_map<pair<uint, uint>, double> pathLength; //= { {cell_start, 0.0} };
        //TODO(melkonyan): this is potentially very unoptimal, because poses will be copied many times.
        unordered_map<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> parents;
        geometry_msgs::PoseStamped reached_pose;
        bool reached_goal = false;

        while (!candidatePoses.empty()) {
            PoseWithDist cand = *candidatePoses.begin();
            auto cell_cand = getCell(cand.pose);
            if (cell_cand == cell_goal) {
                reached_pose = cand.pose;
                reached_goal = true;
                break;
            }
            double l_cand = pathLength[cell_cand];

            candidatePoses.erase(cand);
            vector<PoseWithDist> neighbors = getNeighbors(cand.pose);
            for (auto &nbr : neighbors) {
                auto cell_nbr = getCell(nbr.pose);
                if (cell_cand == cell_nbr) {
                    ROS_WARN("AStarPlanner: Oops, ended up in the same cell.");
                }
                if (costmap_->getCost(cell_nbr.first, cell_nbr.second) > 0) {
                    continue;
                }
                if (pathLength.find(cell_nbr) == pathLength.end() || l_cand + nbr.dist > pathLength[cell_nbr]) {
                    pathLength[cell_nbr] = l_cand + nbr.dist;
                    parents[nbr.pose] = cand.pose;
                    candidatePoses.insert(PoseWithDist(l_cand + nbr.dist + distEstimate(nbr.pose), cand.pose));
                }

            }
        }
        if (reached_goal) {
            getPath(parents, plan);
        }
        return reached_goal;

    }

    void AStarPlanner::getPath(const unordered_map<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> &parents,
            vector<geometry_msgs::PoseStamped> &path) {

    }

    vector<PoseWithDist>& AStarPlanner::getNeighbors(const geometry_msgs::PoseStamped &pose) {

    }

    pair<uint, uint> AStarPlanner::getCell(const geometry_msgs::PoseStamped &pos) {

    }
}
