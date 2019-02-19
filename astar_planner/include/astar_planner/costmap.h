//
// Created by alex on 16.02.19.
//

#ifndef ASTAR_PLANNER_COSTMAP_H
#define ASTAR_PLANNER_COSTMAP_H

#include <costmap_2d/costmap_2d.h>

namespace astar_planner {

    class Costmap {
    public:
        virtual unsigned getCost(unsigned int x, unsigned int y) const = 0;

        virtual double getSizeInMetersX() const = 0;

        virtual double getSizeInMetersY() const = 0;

        virtual unsigned int getSizeInCellsX() const = 0;

        virtual unsigned int getSizeInCellsY() const = 0;

        virtual bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const = 0;

        virtual ~Costmap() = default;
    };

    class EmptyCostmap : public Costmap {
    private:
        unsigned int sizeX;
        unsigned int sizeY;
        unsigned int resolution;
    public:
        EmptyCostmap(unsigned int sizeX, unsigned int sizeY, unsigned int resolution);

        unsigned int getCost(unsigned int x, unsigned int y) const override;

        double getSizeInMetersX() const override;

        double getSizeInMetersY() const override;

        unsigned int getSizeInCellsX() const override;

        unsigned int getSizeInCellsY() const override;

        /**
          * @brief  Convert from world coordinates to map coordinates
          * @param  wx The x world coordinate
          * @param  wy The y world coordinate
          * @param  mx Will be set to the associated map x coordinate
          * @param  my Will be set to the associated map y coordinate
          * @return True if the conversion was successful (legal bounds) false otherwise
          */
        bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const override;

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
