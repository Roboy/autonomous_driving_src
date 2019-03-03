//
// Created by alex on 15.02.19.
//

#ifndef ASTAR_PLANNER_UTILS_H
#define ASTAR_PLANNER_UTILS_H

#include <geometry_msgs/PoseStamped.h>
#include <ostream>

namespace astar_planner {

    /**
     * Container for storing pose of a 2D object in space.
     */
    struct Pose {
        double x;
        double y;
        double th;

        Pose();

        Pose(double x, double y, double th);

        Pose(const geometry_msgs::PoseStamped &pose);

        geometry_msgs::PoseStamped toPoseStamped();

        bool operator==(const Pose &other) const;

        friend std::ostream& operator<<(std::ostream &out, const Pose &pos);

    };

    /**
     * Container for storing object position and associated cost.
     */
    struct PoseWithDist {
        double dist;
        Pose pose;

        PoseWithDist(double dist, const Pose &pose) : dist(dist), pose(pose) {}

        bool operator<(const PoseWithDist &other) const;

        bool operator==(const PoseWithDist &other) const;
    };

    /**
     * Conainer for storing Costmap coordinates.
     */
    struct Cell {
        uint x;
        uint y;

        Cell(): Cell(0, 0) {}

        Cell(uint x, uint y) : x(x), y(y) {}

        bool operator==(const Cell &p) const {
            return x == p.x && y == p.y;
        }
    };

    /**
     * Compute Euclidean distance between two positions.
     */
    double euclid_dist(const Pose &pose1, const Pose &pose2);
}

namespace std {

    template<>
    struct hash<astar_planner::Cell> {
        std::size_t operator()(const astar_planner::Cell &p) const {
            return (p.x << 10) + p.y;
        }
    };

    template<>
    struct hash<astar_planner::Pose> {
        // TODO: think carefully about the hash function
        std::size_t operator()(const astar_planner::Pose &pose) const {
            return (size_t(pose.x) << 20) +
                   (size_t(pose.y) << 10) + size_t(pose.th);
        }
    };

}



#endif //ASTAR_PLANNER_UTILS_H
