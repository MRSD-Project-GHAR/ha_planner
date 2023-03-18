#pragma once
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>

namespace rand_grid_map_gen
{
class RandomMapGen
{
public:
  RandomMapGen(ros::NodeHandle& nh_private);

private:
  ros::NodeHandle nh_private_;
  int seed_;

  int map_width_;
  int map_length_;

  int max_obstacle_length_;
  int max_obstacle_width_;
  int max_obstacle_height_;

  grid_map::GridMap grid_map_;
  grid_map::GridMapRosConverter grid_map_converter_;

};

}  // namespace rand_grid_map_gen