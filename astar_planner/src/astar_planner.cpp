//
// Created by alex on 13.02.19.
//

#include <pluginlib/class_list_macros.h>

#include <chrono>
#include <ctime>
#include <math.h>
#include <ros/console.h>
#include <queue>
#include <unordered_map>

#include <astar_planner/astar_planner.h>
#include <astar_planner/utils.h>

PLUGINLIB_EXPORT_CLASS(astar_planner::AStarPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace astar_planner {

    AStarPlanner::AStarPlanner() :
            name_(""), costmap_(nullptr), step_size_(0.0), turning_radius_(0.0), global_frame_("") {}

    void AStarPlanner::initialize(std::string name,
                                  costmap_2d::Costmap2DROS *costmap_ros) {
        name_ = name;
        costmap_ = new CostmapAdapter(costmap_ros->getCostmap());
        global_frame_ = costmap_ros->getGlobalFrameID();
        loadParameters();
        ROS_INFO("AStarPlanner initialized with name '%s'", name_.c_str());
    };

    void AStarPlanner::initialize(std::string name, astar_planner::Costmap *costmap) {
        name_ = name;
        costmap_ = costmap;
        loadParameters();
        ROS_INFO("AStarPlanner initialized with name '%s'", name_.c_str());
    }

    void AStarPlanner::loadParameters() {
        ros::NodeHandle nh("~" + name_);
        nh.param<double>(std::string("turning_radius"), turning_radius_, 0.0);
        nh.param<double>("step_size", step_size_, 0.0);
    }

    bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                const geometry_msgs::PoseStamped &goal,
                                std::vector<geometry_msgs::PoseStamped> &plan) {
        std::vector<Pose> positions;
        bool foundPlan = false;
        try {
            foundPlan = makePlan(Pose(start), Pose(goal), positions);
        } catch (exception &ex) {
            ROS_FATAL("AStarPlanner exception occured %s", ex.what());
            throw ex;
        }
        if (!foundPlan) {
            return false;
        }
        ros::Time plan_time = ros::Time::now();
        for (auto position : positions) {
            auto pose = position.toPoseStamped();
            pose.header.stamp = plan_time;
            pose.header.frame_id = global_frame_;
            plan.push_back(pose);
        }
        return true;
    }

//    bool AStarPlanner::makePlan(const Pose &start,
//                                const Pose &goal,
//                                std::vector<Pose> &plan) {
//        double start_x = start.x;
//        double start_y = start.y;
//        double start_th = start.th;
//
//        double goal_x = goal.x;
//        double goal_y = goal.y;
//        double goal_th = goal.th;
//
//        int num_points = 2;
//
//        for (int i = 0; i <= num_points; i++) {
//            double x = start_x + (goal_x - start_x) / num_points * i;
//            double y = start_y + (goal_y - start_y) / num_points * i;
//            double th = start_th + (goal_th - start_th) / num_points * i;
//            plan.push_back(Pose(x, y, th));
//        }
//
//        return true;
//    }

    bool AStarPlanner::makePlan(const Pose &start, const Pose &goal, vector<Pose> &path) {
        if (!validateParameters()) {
            return false;
        }
        auto cell_start = getCell(start);
        auto cell_goal = getCell(goal);
        // Instantiate data structures
        set<PoseWithDist> candidatePoses = {PoseWithDist(0.0, start)};
        unordered_map<Cell, double> pathLength = {{cell_start, 0.0}};
        //TODO(melkonyan): this is potentially very unoptimal, because poses will be copied many times.
        unordered_map<Pose, Pose> parents;
        Pose reached_pose;
        bool reached_goal = false;

        // Create variables to log performance
        auto start_time = chrono::system_clock::now();
        int num_nodes_visited = 0;

        while (!candidatePoses.empty()) {
            num_nodes_visited++;
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
                if (!checkBounds(nbr.pose)) {
                    continue;
                }
                auto cell_nbr = getCell(nbr.pose);
                if (cell_cand == cell_nbr) {
                    ROS_WARN("AStarPlanner: Oops, ended up in the same cell.");
                    continue;
                }
                if (costmap_->getCost(cell_nbr.x, cell_nbr.y) > 0) {
                    continue;
                }
                if (pathLength.find(cell_nbr) == pathLength.end() || l_cand + nbr.dist < pathLength[cell_nbr]) {
                    pathLength[cell_nbr] = l_cand + nbr.dist;
                    parents[nbr.pose] = cand.pose;
                    candidatePoses.insert(PoseWithDist(l_cand + nbr.dist + distEstimate(nbr.pose, goal), nbr.pose));
                }
            }
        }
        if (reached_goal) {
            getPath(parents, reached_pose, path);
        }
        auto end_time = chrono::system_clock::now();
        chrono::duration<double> spent_time = end_time - start_time;
        ROS_INFO("AStarPlanner finished in %.2fs, generated %d nodes", spent_time.count(), num_nodes_visited);
        return reached_goal;

    }

    bool AStarPlanner::validateParameters() const {
        if (turning_radius_ <= 0) {
            ROS_ERROR("AStarPlanner: turning radius has invalid value=%.2f. Must be greater than zero.",
                      turning_radius_);
            return false;
        }
        if (step_size_ <= 0) {
            ROS_ERROR("AStarPlanner: step size has invalid value=%.2f. Must be greater than zero.",
                      step_size_);
            return false;
        }
        double dth = step_size_ / turning_radius_;
        if ((cos(0) - cos(dth)) * turning_radius_ <= 1.0 * costmap_->getResolution()) {
            ROS_ERROR("AStarPlanner: provided step size=%.2f is too small for the given turning radius=%.2f and map resolution=%.2f",
                      step_size_, turning_radius_, costmap_->getResolution());
            return false;
        }
        return true;
    }

    void AStarPlanner::getPath(const unordered_map<Pose, Pose> &parents,
                               const Pose &goal_pose,
                               vector<Pose> &path) const {
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


    PoseWithDist AStarPlanner::turnLeft(const Pose &pos, double dth) const {
        double pos_dx = -sin(pos.th) * turning_radius_;
        double pos_dy = cos(pos.th) * turning_radius_;
        double c_x = pos.x - pos_dx;
        double c_y = pos.y - pos_dy;
        double new_dx = -sin(pos.th - dth) * turning_radius_;
        double new_dy = cos(pos.th - dth) * turning_radius_;
        auto new_pos = Pose(c_x + new_dx, c_y + new_dy, pos.th - dth);
        return PoseWithDist(step_size_, new_pos);
    }

    PoseWithDist AStarPlanner::goStraight(const Pose &pos) const {
        auto go_straight = Pose();
        go_straight.x = pos.x + step_size_ * cos(pos.th);
        go_straight.y = pos.y + step_size_ * sin(pos.th);
        go_straight.th = pos.th;
        return PoseWithDist(step_size_, go_straight);
    }

    PoseWithDist AStarPlanner::turnRight(const Pose &pos, double dth) const {
        double pos_dx = sin(pos.th) * turning_radius_;
        double pos_dy = -cos(pos.th) * turning_radius_;
        double c_x = pos.x - pos_dx;
        double c_y = pos.y - pos_dy;
        double new_dx = sin(pos.th + dth) * turning_radius_;
        double new_dy = -cos(pos.th + dth) * turning_radius_;
        auto new_pos = Pose(c_x + new_dx, c_y + new_dy, pos.th + dth);
        return PoseWithDist(step_size_, new_pos);
    }

    bool AStarPlanner::checkBounds(const Pose &pos) const {
        uint x, y;
        return costmap_->worldToMap(pos.x, pos.y, x, y);
    }

    vector<PoseWithDist> AStarPlanner::getNeighbors(const Pose &pos) const {
        double dth = step_size_ / turning_radius_;
        return {turnLeft(pos, dth), goStraight(pos), turnRight(pos, dth)};
    }

    Cell AStarPlanner::getCell(const Pose &pos) const {
        Cell cell;
        costmap_->worldToMap(pos.x, pos.y, cell.x, cell.y);
        return cell;
    }

    double AStarPlanner::distEstimate(const Pose &pose1, const Pose &pose2) const {
        return euclid_dist(pose1, pose2);
    }

    AStarPlanner::~AStarPlanner() {
        delete costmap_;
    }
}
