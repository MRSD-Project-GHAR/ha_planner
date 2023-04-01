#include "mbf_rrts_planner/mbf_rrts_core.hpp"

#include <grid_map_ros/grid_map_ros.hpp>

#include <ros/ros.h>
#include <nav_msgs/Path.h>

class PlanVisualizer
{
public:
  PlanVisualizer(ros::NodeHandle nh, ros::NodeHandle nh_private)
  {
    map_sub_ = nh.subscribe("map_topic", 10, &PlanVisualizer::mapCallback, this);
    start_sub_ = nh.subscribe("start_topic", 10, &PlanVisualizer::startPoseCallback, this);
    goal_sub_ = nh.subscribe("goal_topic", 10, &PlanVisualizer::goalPoseCallback, this);

    path_pub_ = nh.advertise<nav_msgs::Path>("planned_path", 10);

    start_.pose.position.x = 0;
    start_.pose.position.y = 1;

    goal_.pose.position.x = 1;
    goal_.pose.position.y = 0;
  }

  void mapCallback(const grid_map_msgs::GridMap& map_msg)
  {
    grid_map::GridMapRosConverter::fromMessage(map_msg, map_);
    planner_.setMapPtr(std::make_shared<grid_map::GridMap>(map_));
    received_map_ = true;
  }

  void startPoseCallback(const geometry_msgs::PoseStamped& start_msg)
  {
    start_ = start_msg;
    if (received_map_)
      makeAndPublishPlan();
  }

  void goalPoseCallback(const geometry_msgs::PoseStamped& goal_msg)
  {
    goal_ = goal_msg;
  }

private:
  void makeAndPublishPlan()
  {
    std::vector<geometry_msgs::PoseStamped> plan;
    double cost;
    std::string message;

    planner_.makePlan(start_, goal_, 0.1, plan, cost, message);

    nav_msgs::Path planned_path;
    planned_path.header.frame_id = "map";
    planned_path.header.stamp = ros::Time::now();

    planned_path.poses = plan;

    path_pub_.publish(planned_path);
  }

  grid_map::GridMap map_;
  mbf_rrts_core::RRTSPlanner planner_;
  mbf_rrts_core::GridMapPtr map_ptr;
  geometry_msgs::PoseStamped start_;
  geometry_msgs::PoseStamped goal_;

  ros::Subscriber map_sub_;
  ros::Subscriber start_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher path_pub_;

  bool received_map_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrts_visualization");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  PlanVisualizer plan_visualizer(nh, nh_private);

  // std::cout << (DBL_MAX);

  ros::spin();

  return 0;
}