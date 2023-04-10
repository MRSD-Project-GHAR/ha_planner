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

  nh_private_.param("min_slope", min_slope_, 45.0);
}

void RandomMapGen::generateNewMap()
{
  obstacle_list.clear();
  for (int i = 0; i < num_obstacles_; i++)
  {
    addRandomObstacle();
  }

  populateMap();
}

void RandomMapGen::addRandomObstacle()
{
  // TODO: Add different obstacle orientations

  Obstacle new_obstacle;
  new_obstacle.length = randomGenerator(min_obstacle_length_, max_obstacle_length_);
  new_obstacle.width = randomGenerator(min_obstacle_width_, max_obstacle_width_);
  new_obstacle.height = randomGenerator(min_obstacle_height_, max_obstacle_height_);

  new_obstacle.x = randomGenerator(-map_length_ / 2.0, map_length_ / 2.0);
  new_obstacle.y = randomGenerator(-map_width_ / 2.0, map_width_ / 2.0);

  ROS_INFO("Adding obstacle with dimensions %lf x %lf and height %lf at (%lf, %lf)", new_obstacle.length,
           new_obstacle.width, new_obstacle.height, new_obstacle.x, new_obstacle.y);

  new_obstacle.slope1 = (resolution_ / 2.0) * tanf64((M_PI / 180) * randomGenerator(min_slope_, 90));
  new_obstacle.slope2 = (resolution_ / 2.0) * tanf64((M_PI / 180) * randomGenerator(min_slope_, 90));
  new_obstacle.slope3 = (resolution_ / 2.0) * tanf64((M_PI / 180) * randomGenerator(min_slope_, 90));
  new_obstacle.slope4 = (resolution_ / 2.0) * tanf64((M_PI / 180) * randomGenerator(min_slope_, 90));

  obstacle_list.push_back(new_obstacle);
}

void RandomMapGen::populateMap()
{
  // TODO: Add different obstacle orientations
  grid_map_.erase("elevation");
  grid_map_.add("elevation", 0.0);

  for (int i = 0; i < obstacle_list.size(); i++)
  {
    Obstacle new_obstacle = obstacle_list[i];

    for (float x = -new_obstacle.length / 2.0; x < new_obstacle.length / 2.0; x += resolution_ / 2.0)
    {
      for (float y = -new_obstacle.width / 2.0; y < new_obstacle.width / 2.0; y += resolution_ / 2.0)
      {
        if (grid_map_.isInside({ new_obstacle.x + x, new_obstacle.y + y }))
        {
          grid_map_.atPosition("elevation", { new_obstacle.x + x, new_obstacle.y + y }) = new_obstacle.height;
        }
      }
    }

    for (float x = -new_obstacle.length / 2.0; x < new_obstacle.length / 2.0; x += resolution_ / 2.0)
    {
      double current_height = new_obstacle.height;
      double y = -new_obstacle.width / 2.0;
      while (current_height > 0)
      {
        if (grid_map_.isInside({ new_obstacle.x + x, new_obstacle.y + y }))
        {
          grid_map_.atPosition("elevation", { new_obstacle.x + x, new_obstacle.y + y }) = current_height;
        }

        y -= resolution_ / 2.0;
        current_height -= new_obstacle.slope1;
      }
    }

    for (float x = -new_obstacle.length / 2.0; x < new_obstacle.length / 2.0; x += resolution_ / 2.0)
    {
      double current_height = new_obstacle.height;
      double y = new_obstacle.width / 2.0;
      while (current_height > 0)
      {
        if (grid_map_.isInside({ new_obstacle.x + x, new_obstacle.y + y }))
        {
          grid_map_.atPosition("elevation", { new_obstacle.x + x, new_obstacle.y + y }) = current_height;
        }

        y += resolution_ / 2.0;
        current_height -= new_obstacle.slope2;
      }
    }

    for (float y = -new_obstacle.width / 2.0; y < new_obstacle.width / 2.0; y += resolution_ / 2.0)
    {
      double x = -new_obstacle.length / 2.0;
      double current_height = new_obstacle.height;
      while (current_height > 0)
      {
        if (grid_map_.isInside({ new_obstacle.x + x, new_obstacle.y + y }))
        {
          grid_map_.atPosition("elevation", { new_obstacle.x + x, new_obstacle.y + y }) = current_height;
        }

        x -= resolution_ / 2.0;
        current_height -= new_obstacle.slope3;
      }
    }

    for (float y = -new_obstacle.width / 2.0; y < new_obstacle.width / 2.0; y += resolution_ / 2.0)
    {
      double x = new_obstacle.length / 2.0;
      double current_height = new_obstacle.height;
      while (current_height > 0)
      {
        if (grid_map_.isInside({ new_obstacle.x + x, new_obstacle.y + y }))
        {
          grid_map_.atPosition("elevation", { new_obstacle.x + x, new_obstacle.y + y }) = current_height;
        }

        x += resolution_ / 2.0;
        current_height -= new_obstacle.slope4;
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


void RandomMapGen::addObstacle(Obstacle new_obs) {
  obstacle_list.push_back(new_obs);
  populateMap();
}

void RandomMapGen::deleteObstacle(int index) {
  obstacle_list.erase(obstacle_list.begin() + index);
  populateMap();
}

}  // namespace rand_grid_map_gen
