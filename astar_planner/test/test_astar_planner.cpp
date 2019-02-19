#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <vector>

#include "astar_planner/astar_planner.h"
#include "astar_planner/costmap.h"
#include "astar_planner/utils.h"

using ::testing::ElementsAre;
using astar_planner::Position;

TEST(AStarPlannerTest, testMakePlan) {


    astar_planner::AStarPlanner planner = astar_planner::AStarPlanner();
    astar_planner::Costmap *costmap = new astar_planner::EmptyCostmap(10, 10, 1);
    planner.initialize("test_planner", costmap);
    std::vector<Position> plan;
    ASSERT_TRUE(planner.makePlan(Position(0, 0, 0), Position(2, M_PI + 2, M_PI / 2), plan));
    ASSERT_THAT(plan, ElementsAre(Position(0.0, 0.0, 0.0)));
}

TEST(AStarPlannerTest, testGetNeighbors) {

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}