//
// Created by alex on 15.02.19.
//

#ifndef ASTAR_PLANNER_UTILS_H
#define ASTAR_PLANNER_UTILS_H

#include <geometry_msgs/PoseStamped.h>

namespace astar_planner {


    struct Position {
        double x;
        double y;
        double th;

        Position();

        Position(double x, double y, double th);

        Position(const geometry_msgs::PoseStamped &pose);

        geometry_msgs::PoseStamped toPoseStamped();

        bool operator==(const Position &other) const;

    };


    struct PosWithDist {
        double dist;
        Position pose;

        PosWithDist(double dist, const Position &pose) : dist(dist), pose(pose) {}

        bool operator<(const PosWithDist &other) const;

        bool operator==(const PosWithDist &other) const;
    };


    struct Cell {
        uint x;
        uint y;

        Cell(): Cell(0, 0) {}

        Cell(uint x, uint y) : x(x), y(y) {}

        bool operator==(const Cell &p) const {
            return x == p.x && y == p.y;
        }
    };

    double euclid_dist(const Position &pose1, const Position &pose2);
}

namespace std {

    template<>
    struct hash<astar_planner::Cell> {
        std::size_t operator()(const astar_planner::Cell &p) const {
            return (p.x << 10) + p.y;
        }
    };

    template<>
    struct hash<astar_planner::Position> {
        // TODO: think carefully about the hash function
        std::size_t operator()(const astar_planner::Position &pose) const {
            return (size_t(pose.x) << 20) +
                   (size_t(pose.y) << 10) + size_t(pose.th);
        }
    };

}



#endif //ASTAR_PLANNER_UTILS_H
