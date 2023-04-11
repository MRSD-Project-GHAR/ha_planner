#pragma once
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <std_srvs/Empty.h>
#include <string>

#include <yaml-cpp/yaml.h>

namespace rand_grid_map_gen
{
struct Obstacle
{
  double length;
  double width;
  double height;
  double slope;
  double roughness;
  double orientation;

  double slope1;
  double slope2;
  double slope3;
  double slope4;

  double x;
  double y;

  std::string name;

};

// TODO: Do checking of slope parameter: want to avoid iterating over cells for a long time.
// TODO: Add dynamic adding and changing of obstacles from some external interface
// TODO: Add walls
// TODO: Add saving and loading maps from memory
// TODO: Add dynamic changing of other parameters as well
class RandomMapGen
{
public:
  RandomMapGen(ros::NodeHandle& nh_private);

  grid_map_msgs::GridMap getROSMessage();
  void generateNewMap();
  bool resetMapServiceCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);

  void addObstacle(Obstacle new_obst);
  void deleteObstacle(std::string name);

  Obstacle getObstacle(std::string name);

  void changeObstacle(Obstacle obs);

  inline Obstacle getObstacle(int index)
  {
    return obstacle_list[index];
  }

  inline int getNumObstacles()
  {
    return obstacle_list.size();
  }

  void saveMap(std::string name);

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

  double min_slope_;

  double resolution_;
  int num_obstacles_;

  grid_map::GridMap grid_map_;
  grid_map::GridMapRosConverter grid_map_converter_;

  std::vector<Obstacle> obstacle_list;

  std::string yaml_savepath;

  void loadParams();
  void addRandomObstacle();

  inline double angleToLengthDecrement(double angle)
  {
    return (resolution_ / 2.0) * tanf64((M_PI / 180) * angle);
  }

  inline double randomGenerator(double min, double max)
  {
    return min + ((1.0 + (rand() % 1000)) / 1000.0) * (max - min);
  }

  void populateMap();
};

}  // namespace rand_grid_map_gen