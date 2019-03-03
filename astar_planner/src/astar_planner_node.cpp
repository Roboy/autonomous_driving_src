#include <astar_planner/astar_planner_visualizer.h>
#include <ros/ros.h>
#include <ros/console.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "astar_planner_node");

    astar_planner::AStarPlannerVisualizer visualizer;
    visualizer.initialize();
    ROS_INFO("AStarPlannerNode initialized.");

    ros::spin();
    return 0;
}
