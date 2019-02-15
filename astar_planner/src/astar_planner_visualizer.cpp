/**
 * An example node that will create a costmap and start listening to initial_pose and goal topics.
 */

#include <astar_planner/astar_planner_visualizer.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>

namespace astar_planner {

    AStarPlannerVisualizer::AStarPlannerVisualizer(): costmap_(NULL) {}

    void AStarPlannerVisualizer::initialize() {
        // publish initial position of the robot.
        //geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg(new geometry_msgs::PoseWithCovarianceStamped());
        //updateInitialPosition(msg);

        // Subscribe to initialposition changes
        //ros::Subscriber sub = handle_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1,
        //        boost::bind(&AStarPlannerVisualizer::updateInitialPosition, this, _1));

        // Create a static costmap.
        tf::TransformListener tf(ros::Duration(10));
        costmap_ = new costmap_2d::Costmap2DROS("costmap", tf);
        ROS_INFO("Costmap created");
        costmap_->start();
        ROS_INFO("Costmap started");
        planner_.initialize("astar_planner", costmap_);

        // Publisher for found paths
        plan_publisher_ = handle_.advertise<nav_msgs::Path>("global_plan", 10);
    }

    void AStarPlannerVisualizer::updateInitialPosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        // Parse received position
        geometry_msgs::Point position = msg->pose.pose.position;
        geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
        ROS_INFO("Handling new initial position (%.2f, %.2f, %.2f)", position.x, position.y, orientation.z);
        // Create corresponding transform message.
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(position.x, position. y, 0.0) );
        transform.setRotation( tf::Quaternion(0, 0, orientation.z, orientation.w) );

        // Broadcast transform message
        static tf::TransformBroadcaster tf_broadcaster;
        tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "base_link"));
    }

    void AStarPlannerVisualizer::updateGoal(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        geometry_msgs::PoseStamped goal_pose = *msg;
        geometry_msgs::PoseStamped start_pose = goal_pose;
        start_pose.pose.position = geometry_msgs::Point();
        std::vector<geometry_msgs::PoseStamped> plan;
        if (!planner_.makePlan(start_pose, goal_pose, plan)) {
            ROS_ERROR("AStarPlanner failed to create a plan");
        }
        nav_msgs::Path plan_msg;
        plan_msg.poses = plan;
        plan_publisher_.publish(plan_msg);
    }

    AStarPlannerVisualizer::~AStarPlannerVisualizer() {
        ROS_INFO("Destructor called.");
        delete costmap_;
    }
}