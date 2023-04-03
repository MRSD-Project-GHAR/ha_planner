#include "rand_grid_map_gen/map_gen.hpp"

namespace rand_grid_map_gen
{
RandomMapGen::RandomMapGen(ros::NodeHandle& nh_private)
{
  nh_private_ = nh_private;

  loadParams();

  grid_map_.setBasicLayers({ "elevation" });
  grid_map_.setGeometry({ map_length_, map_width_ }, resolution_);
  grid_map_.setFrameId("map");

  reset_map_service_ = nh_private_.advertiseService("reset_map", &RandomMapGen::resetMapServiceCallback, this);

  if (seed_ == 0)
    srand(ros::Time::now().toNSec());

  else
    srand(seed_);

  generateNewMap();
}

void RandomMapGen::loadParams()
{
  // TODO: add more parameters for slope and roughness
  nh_private_.param("seed", seed_, 0);

  nh_private_.param("map_length", map_length_, 10.0);
  nh_private_.param("map_width", map_width_, 10.0);

  nh_private_.param("max_obstacle_length", max_obstacle_length_, 1.0);
  nh_private_.param("max_obstacle_width", max_obstacle_width_, 1.0);
  nh_private_.param("max_obstacle_height", max_obstacle_height_, 2.0);

  nh_private_.param("min_obstacle_length", min_obstacle_length_, 0.5);
  nh_private_.param("min_obstacle_width", min_obstacle_width_, 0.5);
  nh_private_.param("min_obstacle_height", min_obstacle_height_, 2.0);

  nh_private_.param("resolution", resolution_, 0.1);
  nh_private_.param("num_obstacles", num_obstacles_, 2);
}

void RandomMapGen::generateNewMap()
{
  grid_map_.erase("elevation");
  grid_map_.add("elevation", 0.0);

  for (int i = 0; i < num_obstacles_; i++)
  {
    addObstacle();
  }
}

void RandomMapGen::addObstacle()
{
  // TODO: Add different obstacle orientations
  double obs_length =
      min_obstacle_length_ + ((1.0 + (rand() % 1000)) / 1000.0) * (max_obstacle_length_ - min_obstacle_length_);
  double obs_width =
      min_obstacle_width_ + ((1.0 + (rand() % 1000)) / 1000.0) * (max_obstacle_width_ - min_obstacle_width_);
  double obs_height =
      min_obstacle_height_ + ((1.0 + (rand() % 1000)) / 1000.0) * (max_obstacle_height_ - min_obstacle_height_);

  double obs_x = (-map_length_ / 2.0) + (((1.0 + (rand() % 1000)) / 1000.0) * map_length_);
  double obs_y = (-map_width_ / 2.0) + (((1.0 + (rand() % 1000)) / 1000.0) * map_width_);

  ROS_INFO("Adding obstacle with dimensions %lf x %lf and height %lf at (%lf, %lf)", obs_length, obs_width, obs_height,
           obs_x, obs_y);

  for (float x = -obs_length / 2.0; x < obs_length / 2.0; x += resolution_ / 2.0)
  {
    for (float y = -obs_width / 2.0; y < obs_width / 2.0; y += resolution_ / 2.0)
    {
      if (grid_map_.isInside({ obs_x + x, obs_y + y }))
      {
        grid_map_.atPosition("elevation", { obs_x + x, obs_y + y }) = obs_height;
      }
    }
  }
}

grid_map_msgs::GridMap RandomMapGen::getROSMessage()
{
  grid_map_msgs::GridMap msg;
  grid_map_converter_.toMessage(grid_map_, msg);
  return msg;
}

bool RandomMapGen::resetMapServiceCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
  loadParams();
  generateNewMap();
  return true;
}

}  // namespace rand_grid_map_gen
