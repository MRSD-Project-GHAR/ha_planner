#include "rand_grid_map_gen/map_gen.hpp"
#include <fstream>

namespace rand_grid_map_gen
{
RandomMapGen::RandomMapGen(ros::NodeHandle& nh_private)
{
  nh_private_ = nh_private;

  loadParams();

  grid_map_.setBasicLayers({ "elevation" });
  grid_map_.setGeometry({ map_length_, map_width_ }, resolution_);
  grid_map_.setFrameId("map");

  nh_private.param("yaml_file_name", yaml_savepath, std::string("/opt/ros/"));

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

  static int number = 1;

  Obstacle new_obstacle;
  new_obstacle.length = randomGenerator(min_obstacle_length_, max_obstacle_length_);
  new_obstacle.width = randomGenerator(min_obstacle_width_, max_obstacle_width_);
  new_obstacle.height = randomGenerator(min_obstacle_height_, max_obstacle_height_);

  new_obstacle.x = randomGenerator(-map_length_ / 2.0, map_length_ / 2.0);
  new_obstacle.y = randomGenerator(-map_width_ / 2.0, map_width_ / 2.0);

  ROS_INFO("Adding obstacle with dimensions %lf x %lf and height %lf at (%lf, %lf)", new_obstacle.length,
           new_obstacle.width, new_obstacle.height, new_obstacle.x, new_obstacle.y);

  new_obstacle.slope1 = randomGenerator(min_slope_, 90);
  new_obstacle.slope2 = randomGenerator(min_slope_, 90);
  new_obstacle.slope3 = randomGenerator(min_slope_, 90);
  new_obstacle.slope4 = randomGenerator(min_slope_, 90);

  new_obstacle.name = "Random Obstacle " + std::to_string(number);
  number++;
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
        current_height -= angleToLengthDecrement(new_obstacle.slope1);
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
        current_height -= angleToLengthDecrement(new_obstacle.slope2);
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
        current_height -= angleToLengthDecrement(new_obstacle.slope3);
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
        current_height -= angleToLengthDecrement(new_obstacle.slope4);
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

void RandomMapGen::addObstacle(Obstacle new_obs)
{
  obstacle_list.push_back(new_obs);
  populateMap();
}

void RandomMapGen::changeObstacle(Obstacle obs)
{
  for (auto it = obstacle_list.begin(); it != obstacle_list.end(); it++)
  {
    if (it->name == obs.name)
    {
      *it = obs;
      break;
    }
  }

  populateMap();
}

void RandomMapGen::deleteObstacle(std::string name)
{
  for (auto it = obstacle_list.begin(); it != obstacle_list.end(); it++)
  {
    if (it->name == name)
    {
      obstacle_list.erase(it);
      break;
    }
  }

  populateMap();
}

Obstacle RandomMapGen::getObstacle(std::string name)
{
  for (auto it = obstacle_list.begin(); it != obstacle_list.end(); it++)
  {
    if (it->name == name)
    {
      return *it;
    }
  }

  Obstacle obs;
  obs.name = "No obstacle found";
  return obs;
}

void RandomMapGen::saveMap(std::string name)
{
  static int count = 0;
  YAML::Emitter em;
  em << YAML::BeginMap;
  em << YAML::Key << "Obstacles";
  em << YAML::Value;
  em << YAML::BeginSeq;

  for (int i = 0; i < obstacle_list.size(); i++)
  {
    em << YAML::BeginMap;

    em << YAML::Key << "x" << YAML::Value << obstacle_list[i].x;
    em << YAML::Key << "y" << YAML::Value << obstacle_list[i].y;

    em << YAML::Key << "height" << YAML::Value << obstacle_list[i].height;
    em << YAML::Key << "width" << YAML::Value << obstacle_list[i].width;
    em << YAML::Key << "length" << YAML::Value << obstacle_list[i].length;

    em << YAML::Key << "slope1" << YAML::Value << obstacle_list[i].slope1;
    em << YAML::Key << "slope2" << YAML::Value << obstacle_list[i].slope2;
    em << YAML::Key << "slope3" << YAML::Value << obstacle_list[i].slope3;
    em << YAML::Key << "slope4" << YAML::Value << obstacle_list[i].slope4;

    em << YAML::Key << "roughness" << YAML::Value << obstacle_list[i].roughness;
    em << YAML::Key << "orientation" << YAML::Value << obstacle_list[i].orientation;

    em << YAML::EndMap;
  }

  em << YAML::EndSeq;
  em << YAML::EndMap;

  std::cout << em.c_str();

  std::ofstream myfile;
  myfile.open(yaml_savepath + name);
  myfile << em.c_str();
  myfile.close();
}

}  // namespace rand_grid_map_gen
