//
// Created by alex on 16.02.19.
//

#ifndef ASTAR_PLANNER_COSTMAP_H
#define ASTAR_PLANNER_COSTMAP_H

#include <costmap_2d/costmap_2d.h>

namespace astar_planner {

    class Costmap {
    public:
        virtual unsigned getCost(unsigned int x, unsigned int y) = 0;

        virtual double getSizeInMetersX() = 0;

        virtual double getSizeInMetersY() = 0;

        virtual unsigned int getSizeInCellsX() = 0;

        virtual unsigned int getSizeInCellsY() = 0;

        virtual ~Costmap() = default;
    };

    class EmptyCostmap : public Costmap {
    private:
        unsigned int sizeX;
        unsigned int sizeY;
        unsigned int resolution;
    public:
        EmptyCostmap(unsigned int sizeX, unsigned int sizeY, unsigned int resolution);

        unsigned int getCost(unsigned int x, unsigned int y) override;

        double getSizeInMetersX() override;

        double getSizeInMetersY() override;

        unsigned int getSizeInCellsX() override;

        unsigned int getSizeInCellsY() override;

        ~EmptyCostmap() override;
    };

    class CostmapAdapter : public Costmap {
    private:
        costmap_2d::Costmap2D *costmap_;
    public:
        CostmapAdapter(costmap_2d::Costmap2D costmap);

        unsigned int getCost();

        double getSizeInMetersX();

        double getSizeInMetersY();

        unsigned int getSizeInCellsX();

        unsigned int getSizeInCellsY();
    };
}
#endif //ASTAR_PLANNER_COSTMAP_H
