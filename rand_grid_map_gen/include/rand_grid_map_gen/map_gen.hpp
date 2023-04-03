#pragma once
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <std_srvs/Empty.h>

namespace rand_grid_map_gen
{
class RandomMapGen
{
public:
  RandomMapGen(ros::NodeHandle& nh_private);

  grid_map_msgs::GridMap getROSMessage();
  void generateNewMap();
  bool resetMapServiceCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);

private:
  ros::NodeHandle nh_private_;
  ros::ServiceServer reset_map_service_;

  int seed_;

  double map_width_;
  double map_length_;

  double max_obstacle_length_;
  double max_obstacle_width_;
  double max_obstacle_height_;

  double min_obstacle_length_;
  double min_obstacle_width_;
  double min_obstacle_height_;

  double resolution_;
  int num_obstacles_;

  grid_map::GridMap grid_map_;
  grid_map::GridMapRosConverter grid_map_converter_;

  void loadParams();
  void addObstacle();


};

}  // namespace rand_grid_map_gen