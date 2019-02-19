#include "astar_planner/costmap.h"

namespace astar_planner {

    EmptyCostmap::EmptyCostmap(unsigned int sizeX, unsigned int sizeY, unsigned int resolution)
            : sizeX(sizeX), sizeY(sizeY), resolution(resolution) {}

    unsigned int EmptyCostmap::getCost(unsigned int x, unsigned int y) const {
        return 0;
    }

    double EmptyCostmap::getSizeInMetersX() const {
        return double(sizeX) / resolution;
    }

    double EmptyCostmap::getSizeInMetersY() const {
        return double(sizeY) / resolution;
    }

    unsigned int EmptyCostmap::getSizeInCellsX() const {
        return sizeX;
    }

    unsigned int EmptyCostmap::getSizeInCellsY() const {
        return sizeY;
    }

    bool EmptyCostmap::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const {
        uint x = uint(wx * resolution);
        uint y = uint(wy * resolution);
        if (x < 0 || x >= getSizeInCellsX() || y < 0 || y >= getSizeInCellsY()) {
            return false;
        }
        mx = x;
        my = y;
        return true;
    }

    EmptyCostmap::~EmptyCostmap() = default;
}
