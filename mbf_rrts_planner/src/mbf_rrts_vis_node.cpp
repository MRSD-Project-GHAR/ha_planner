#include "mbf_rrts_planner/mbf_rrts_core.hpp"

#include <grid_map_ros/grid_map_ros.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <mbf_rrts_planner/mbf_rrts_plannerConfig.h>

#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>

class PlanVisualizer
{
public:
  PlanVisualizer(ros::NodeHandle nh, ros::NodeHandle nh_private)
  {
    map_sub_a_ = nh.subscribe("map_topic_robot_A", 10, &PlanVisualizer::mapACallback, this);
    map_sub_b_ = nh.subscribe("map_topic_robot_B", 10, &PlanVisualizer::mapBCallback, this);
    start_sub_ = nh.subscribe("start_topic", 10, &PlanVisualizer::startPoseCallback, this);
    goal_sub_ = nh.subscribe("goal_topic", 10, &PlanVisualizer::goalPoseCallback, this);

    path_pub_A_ = nh.advertise<nav_msgs::Path>("planned_path_A", 10);
    path_pub_B_ = nh.advertise<nav_msgs::Path>("planned_path_B", 10);
    path_pub_C_ = nh.advertise<nav_msgs::Path>("planned_path_C", 10);

    plan_service_ = nh.advertiseService("generate_plan", &PlanVisualizer::planServiceCallback, this);

    start_.pose.position.x = 0;
    start_.pose.position.y = 1;

    goal_.pose.position.x = 1;
    goal_.pose.position.y = 0;
  }

  void mapACallback(const grid_map_msgs::GridMap& map_msg)
  {
    grid_map::GridMapRosConverter::fromMessage(map_msg, map_a_);
    planner_.setMapPtr(std::make_shared<grid_map::GridMap>(map_a_));
    received_map_a_ = true;
  }

  void mapBCallback(const grid_map_msgs::GridMap& map_msg)
  {
    grid_map::GridMapRosConverter::fromMessage(map_msg, map_b_);
    planner_.setMapPtr(std::make_shared<grid_map::GridMap>(map_b_));
    received_map_b_ = true;
  }

  void startPoseCallback(const geometry_msgs::PoseStamped& start_msg)
  {
    start_ = start_msg;
  }

  void goalPoseCallback(const geometry_msgs::PoseStamped& goal_msg)
  {
    goal_ = goal_msg;
  }

  void reconfigureCallback(mbf_rrts_planner::mbf_rrts_plannerConfig& config, uint32_t level)
  {
    // planner_.setParams(config);
    planner_.setDistanceFactor(config.distance_factor);
    planner_.setIterations(config.iterations);
    planner_.setSeed(config.seed);
    planner_.setNeighbourhoodSize(config.neighbourhood_size);
  }

  void publishPlan()
  {
    nav_msgs::Path planned_path;
    planned_path.header.frame_id = "map";
    planned_path.header.stamp = ros::Time::now();

    // TODO: add some height offset to the path so that it is visible in rviz
    planned_path.poses = plan_A_;
    path_pub_A_.publish(planned_path);
    planned_path.poses = plan_B_;
    path_pub_B_.publish(planned_path);
    // planned_path.poses = plan_C_;
    // path_pub_C_.publish(planned_path);
  }

  bool planServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    if (received_map_a_ && received_map_b_)
    {
      makeAndPublishPlan();
      return true;
    }
    else
    {
      return false;
    }
  }

private:
  void makeAndPublishPlan()
  {
    std::vector<geometry_msgs::PoseStamped> plan;
    double cost;
    std::string message;
    plan_A_.clear();
    plan_B_.clear();
    plan_C_.clear();

    planner_.setLayerName("traversability_A");
    planner_.setMapPtr(std::make_shared<grid_map::GridMap>(map_a_));
    planner_.makePlan(start_, goal_, 0.1, plan_A_, cost, message);

    planner_.setLayerName("traversability_B");
    planner_.setMapPtr(std::make_shared<grid_map::GridMap>(map_b_));
    planner_.makePlan(start_, goal_, 0.1, plan_B_, cost, message);
    
    // planner_.setLayerName("traversability_B");
    // planner_.makePlan(start_, goal_, 0.1, plan_C_, cost, message);
    
  }

  grid_map::GridMap map_a_;
  grid_map::GridMap map_b_;
  mbf_rrts_core::RRTSPlanner planner_;
  geometry_msgs::PoseStamped start_;
  geometry_msgs::PoseStamped goal_;

  ros::Subscriber map_sub_a_;
  ros::Subscriber map_sub_b_;
  ros::Subscriber start_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher path_pub_A_;
  ros::Publisher path_pub_B_;
  ros::Publisher path_pub_C_;
  ros::ServiceServer plan_service_;

  std::vector<geometry_msgs::PoseStamped> plan_A_;
  std::vector<geometry_msgs::PoseStamped> plan_B_;
  std::vector<geometry_msgs::PoseStamped> plan_C_;
  bool received_map_a_;
  bool received_map_b_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrts_visualization");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  PlanVisualizer plan_visualizer(nh, nh_private);
  ros::Rate loop_rate(10);

  dynamic_reconfigure::Server<mbf_rrts_planner::mbf_rrts_plannerConfig> server;
  dynamic_reconfigure::Server<mbf_rrts_planner::mbf_rrts_plannerConfig>::CallbackType f;

  f = boost::bind(&PlanVisualizer::reconfigureCallback, &plan_visualizer, _1, _2);
  server.setCallback(f);
  // std::cout << (DBL_MAX);
  while (ros::ok())
  {
    plan_visualizer.publishPlan();
    ros::spinOnce();
    loop_rate.sleep();
  }
  // ros::spin();

  return 0;
}