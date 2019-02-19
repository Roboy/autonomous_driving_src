//
// Created by alex on 13.02.19.
//

#include <pluginlib/class_list_macros.h>

#include <math.h>
#include <ros/console.h>
#include <queue>
#include <unordered_map>

#include <astar_planner/astar_planner.h>
#include <astar_planner/utils.h>

PLUGINLIB_EXPORT_CLASS(astar_planner::AStarPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace astar_planner {

    AStarPlanner::AStarPlanner() : name_(""), costmap_ros_(nullptr), step_size_(0.0), turning_radius_(0.0) {}

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
                                std::vector<geometry_msgs::PoseStamped> &plan) {
        double start_x = start.pose.position.x;
        double start_y = start.pose.position.y;

        double goal_x = goal.pose.position.x;
        double goal_y = goal.pose.position.y;

        int num_points = 2;
        ros::Time plan_time = ros::Time::now();

        for (int i = 0; i <= num_points; i++) {
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
                                   std::vector<geometry_msgs::PoseStamped> &plan) {
        std::vector<Position> positions;
        bool foundPlan = makePlan(Position(start), Position(goal), positions);
        if (!foundPlan) {
            return false;
        }
        for (auto position : positions) {
            plan.push_back(position.toPoseStamped());
        }
        return true;
    }

    bool AStarPlanner::makePlan(const Position &start, const Position &goal, vector<Position> &path) {
        auto cell_start = getCell(start);
        auto cell_goal = getCell(goal);
        set<PosWithDist> candidatePoses;
        unordered_map<Cell, double> pathLength; //= { {cell_start, 0.0} };
        //TODO(melkonyan): this is potentially very unoptimal, because poses will be copied many times.
        unordered_map<Position, Position> parents;
        Position reached_pose;
        bool reached_goal = false;

        while (!candidatePoses.empty()) {
            PosWithDist cand = *candidatePoses.begin();
            auto cell_cand = getCell(cand.pose);
            if (cell_cand == cell_goal) {
                reached_pose = cand.pose;
                reached_goal = true;
                break;
            }
            double l_cand = pathLength[cell_cand];

            candidatePoses.erase(cand);
            vector<PosWithDist> neighbors = getNeighbors(cand.pose);
            for (auto &nbr : neighbors) {
                auto cell_nbr = getCell(nbr.pose);
                if (cell_cand == cell_nbr) {
                    ROS_WARN("AStarPlanner: Oops, ended up in the same cell.");
                }
                if (costmap_->getCost(cell_nbr.x, cell_nbr.y) > 0) {
                    continue;
                }
                if (pathLength.find(cell_nbr) == pathLength.end() || l_cand + nbr.dist < pathLength[cell_nbr]) {
                    pathLength[cell_nbr] = l_cand + nbr.dist;
                    parents[nbr.pose] = cand.pose;
                    candidatePoses.insert(PosWithDist(l_cand + nbr.dist + distEstimate(nbr.pose, goal), cand.pose));
                }

            }
        }
        if (reached_goal) {
            getPath(parents, reached_pose, path);
        }
        return reached_goal;

    }

    void AStarPlanner::getPath(const unordered_map<Position, Position> &parents,
                               const Position &goal_pose,
                               vector<Position> &path) const {
        auto curr_pose = goal_pose;
        while (true) {
            path.push_back(curr_pose);
            auto search = parents.find(curr_pose);
            if (search == parents.end()) {
                break;
            }
            curr_pose = search->second;
        }
        reverse(path.begin(), path.end());
    }

    vector<PosWithDist> AStarPlanner::getNeighbors(const Position &pose) const {
        double dth = step_size_ / turning_radius_;
        auto go_straight = Position();
        go_straight.x = pose.x + step_size_ * cos(dth);
        go_straight.y = pose.y + step_size_ * sin(dth);
        go_straight.th = pose.th;
        return {PosWithDist(step_size_, go_straight)};
        // TODO(melkonyan): implement turning;
    }

    Cell AStarPlanner::getCell(const Position &pos) const {
        Cell cell;
        costmap_->worldToMap(pos.x, pos.y, cell.x, cell.y);
        return cell;
    }

    double
    AStarPlanner::distEstimate(const Position &pose1, const Position &pose2) const {
        return euclid_dist(pose1, pose2);
    }
}
