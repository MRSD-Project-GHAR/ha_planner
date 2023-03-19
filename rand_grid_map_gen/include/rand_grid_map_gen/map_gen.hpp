#pragma once
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>

namespace rand_grid_map_gen
{
class RandomMapGen
{
public:
  RandomMapGen(ros::NodeHandle& nh_private);

  grid_map_msgs::GridMap getROSMessage();
  void generateNewMap();

private:
  ros::NodeHandle nh_private_;
  int seed_;

  double map_width_;
  double map_length_;

  double max_obstacle_length_;
  double max_obstacle_width_;
  double max_obstacle_height_;

  double resolution_;
  int num_obstacles_;

  grid_map::GridMap grid_map_;
  grid_map::GridMapRosConverter grid_map_converter_;

  void addObstacle();

};

}  // namespace rand_grid_map_gen