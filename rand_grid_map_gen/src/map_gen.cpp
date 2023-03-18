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
}





}  // namespace rand_grid_map_gen
