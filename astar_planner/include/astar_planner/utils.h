//
// Created by alex on 15.02.19.
//

#ifndef ASTAR_PLANNER_UTILS_H
#define ASTAR_PLANNER_UTILS_H

#include <geometry_msgs/PoseStamped.h>

namespace astar_planner {

    struct PoseWithDist {
        float dist;
        geometry_msgs::PoseStamped pose;
        PoseWithDist(float dist, const geometry_msgs::PoseStamped &pose): dist(dist), pose(pose) {}

        bool operator<(const PoseWithDist &other) {
            return dist < other.dist;
        }
    };

    struct Position {
        float x;
        float y;
        float th;
        Position(float x, float y, float th): x(x), y(y), th(th) {}
    };

    struct Point {
        float x;
        float y;
        Point(float x, float y): x(x), y(y) {}
    };
}

#endif //ASTAR_PLANNER_UTILS_H
