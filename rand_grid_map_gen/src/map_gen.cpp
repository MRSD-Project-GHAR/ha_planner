#include "rand_grid_map_gen/map_gen.hpp"

namespace rand_grid_map_gen
{

RandomMapGen::RandomMapGen(ros::NodeHandle& nh_private)
{
    nh_private_ = nh_private;

    nh_private_.param("seed", seed_, 0);

    nh_private_.param("map_length", map_length_, 1000);
    nh_private_.param("map_width", map_width_, 1000);

    nh_private_.param("max_obstacle_length", max_obstacle_length_, 20);
    nh_private_.param("max_obstacle_width", max_obstacle_width_, 20);
    nh_private_.param("max_obstacle_height", max_obstacle_height_, 5);

    nh_private_.param("resolution", resolution_, 5);

    grid_map_.setBasicLayers({"elevation"});
    grid_map_.setGeometry({map_length_, map_width_}, resolution_);

    // grid_map_msgs::GridMap msg;

    // grid_map_converter_.toMessage(grid_map_, msg);
    

}

  grid_map_msgs::GridMap RandomMapGen::getROSMessage() {
    grid_map_msgs::GridMap msg;
    grid_map_converter_.toMessage(grid_map_, msg);
    return msg;
  }

}  // namespace rand_grid_map_gen
