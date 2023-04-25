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
    map_sub_ = nh.subscribe("map_topic", 10, &PlanVisualizer::mapCallback, this);
    start_sub_ = nh.subscribe("start_topic", 10, &PlanVisualizer::startPoseCallback, this);
    goal_sub_ = nh.subscribe("goal_topic", 10, &PlanVisualizer::goalPoseCallback, this);

    path_pub_ = nh.advertise<nav_msgs::Path>("planned_path", 10);

    plan_service_ = nh.advertiseService("generate_plan", &PlanVisualizer::planServiceCallback, this);

    start_.pose.position.x = 0;
    start_.pose.position.y = 1;

    goal_.pose.position.x = 1;
    goal_.pose.position.y = 0;

    make_plan_ = false;
    received_map_ = false;
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
    if (make_plan_ && received_map_) {
        // ROS_INFO_STREAM("Making plan now");
        makeAndPublishPlan();
        make_plan_ = false;
    }
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
    planned_path.poses = plan_;
    path_pub_.publish(planned_path);
  }

  bool planServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    make_plan_ = true;
    ROS_INFO_STREAM("Plan service called");
    return true;
  }

private:
  void makeAndPublishPlan()
  {
    std::vector<geometry_msgs::PoseStamped> plan;
    double cost;
    std::string message;
    plan_.clear();

    planner_.setLayerName("traversability");
    planner_.setMapPtr(std::make_shared<grid_map::GridMap>(map_));
    planner_.makePlan(start_, goal_, 0.1, plan_, cost, message);

  }

  grid_map::GridMap map_;
  mbf_rrts_core::RRTSPlanner planner_;
  geometry_msgs::PoseStamped start_;
  geometry_msgs::PoseStamped goal_;

  ros::Subscriber map_sub_;
  ros::Subscriber start_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher path_pub_;
  ros::ServiceServer plan_service_;

  std::vector<geometry_msgs::PoseStamped> plan_;
  bool received_map_;
  bool make_plan_;
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