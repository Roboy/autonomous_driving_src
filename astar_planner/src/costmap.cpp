#include "astar_planner/costmap.h"

namespace astar_planner {

    EmptyCostmap::EmptyCostmap(unsigned int sizeX, unsigned int sizeY, unsigned int resolution)
            : sizeX(sizeX), sizeY(sizeY), resolution(resolution) {}

    unsigned int EmptyCostmap::getCost(unsigned int x, unsigned int y) {
        return 0;
    }

    double EmptyCostmap::getSizeInMetersX() {
        return double(sizeX) / resolution;
    }

    double EmptyCostmap::getSizeInMetersY() {
        return double(sizeY) / resolution;
    }

    unsigned int EmptyCostmap::getSizeInCellsX() {
        return sizeX;
    }

    unsigned int EmptyCostmap::getSizeInCellsY() {
        return sizeY;
    }

    EmptyCostmap::~EmptyCostmap() = default;
}
