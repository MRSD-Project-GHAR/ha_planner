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
  grid_map_.setFrameId("odom");

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
  nh_private_.param("max_obstacle_roughness", max_roughness_, 0.0);
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

  new_obstacle.roughness = randomGenerator(0, max_roughness_);

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

    grid_map::Index bottom_left;
    grid_map::Index top_right;

    grid_map_.getIndex({ new_obstacle.x + new_obstacle.length / 2.0, new_obstacle.y + new_obstacle.width / 2.0 },
                       bottom_left);
    grid_map_.getIndex({ new_obstacle.x - new_obstacle.length / 2.0, new_obstacle.y - new_obstacle.width / 2.0 },
                       top_right);

    int x_left = bottom_left(0);
    int x_right = top_right(0);
    int y_bottom = bottom_left(1);
    int y_top = top_right(1);

    for (int x = x_left; x <= x_right; x++)
    {
      for (float y = y_bottom; y <= y_top; y++)
      {
        if (validIndex({ x, y }))
        {
          grid_map_.at("elevation", { x, y }) =
              new_obstacle.height + randomGenerator(-new_obstacle.roughness, new_obstacle.roughness);
        }
      }
    }

    for (int x = x_left; x <= x_right; x++)
    {
      double current_height = new_obstacle.height;
      int y = y_bottom;
      while (current_height > 0)
      {
        if (validIndex({ x, y }))
        {
          grid_map_.at("elevation", { x, y }) =
              current_height + randomGenerator(-new_obstacle.roughness, new_obstacle.roughness);
        }

        y--;
        current_height -= angleToLengthDecrement(new_obstacle.slope1);
      }
    }

    for (int x = x_left; x <= x_right; x++)
    {
      double current_height = new_obstacle.height;
      int y = y_top;
      while (current_height > 0)
      {
        if (validIndex({ x, y }))
        {
          grid_map_.at("elevation", { x, y }) =
              current_height + randomGenerator(-new_obstacle.roughness, new_obstacle.roughness);
        }

        y++;
        current_height -= angleToLengthDecrement(new_obstacle.slope2);
      }
    }

    for (int y = y_bottom; y <= y_top; y++)
    {
      int x = x_left;
      double current_height = new_obstacle.height;
      while (current_height > 0)
      {
        if (validIndex({ x, y }))
        {
          grid_map_.at("elevation", { x, y }) =
              current_height + randomGenerator(-new_obstacle.roughness, new_obstacle.roughness);
        }

        x--;
        current_height -= angleToLengthDecrement(new_obstacle.slope3);
      }
    }

    for (int y = y_bottom; y <= y_top; y++)
    {
      int x = x_right;
      double current_height = new_obstacle.height;
      while (current_height > 0)
      {
        if (validIndex({ x, y }))
        {
          grid_map_.at("elevation", { x, y }) =
              current_height + randomGenerator(-new_obstacle.roughness, new_obstacle.roughness);
        }

        x++;
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
  em << YAML::Key << "Map_length";
  em << YAML::Value << map_length_;

  em << YAML::Key << "Map_width";
  em << YAML::Value << map_width_;

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

    em << YAML::Key << "name" << YAML::Value << obstacle_list[i].name;

    em << YAML::EndMap;
  }

  em << YAML::EndSeq;
  em << YAML::EndMap;

  std::ofstream myfile;
  myfile.open(yaml_savepath + name);
  myfile << em.c_str();
  myfile.close();
}

void RandomMapGen::loadMap(std::string name)
{
  YAML::Node map_params;
  try
  {
    map_params = YAML::LoadFile(yaml_savepath + name);
  }
  catch (YAML::BadFile e)
  {
    ROS_ERROR_STREAM("This file doesn't exist on disk! Can't load map.");
    return;
  }

  // std::cout << map_params["Obstacles"].size();

  map_length_ = map_params["Map_length"].as<double>();
  map_width_ = map_params["Map_width"].as<double>();

  ROS_INFO("Map length: %lf", map_length_);
  ROS_INFO("Map width: %lf", map_width_);

  grid_map_.setGeometry({ map_length_, map_width_ }, resolution_);

  obstacle_list.clear();

  for (int i = 0; i < map_params["Obstacles"].size(); i++)
  {
    Obstacle obs;

    obs.x = map_params["Obstacles"][i]["x"].as<double>();
    obs.y = map_params["Obstacles"][i]["y"].as<double>();

    obs.width = map_params["Obstacles"][i]["width"].as<double>();
    obs.length = map_params["Obstacles"][i]["length"].as<double>();
    obs.height = map_params["Obstacles"][i]["height"].as<double>();

    obs.slope1 = map_params["Obstacles"][i]["slope1"].as<double>();
    obs.slope2 = map_params["Obstacles"][i]["slope2"].as<double>();
    obs.slope3 = map_params["Obstacles"][i]["slope3"].as<double>();
    obs.slope4 = map_params["Obstacles"][i]["slope4"].as<double>();

    obs.roughness = map_params["Obstacles"][i]["roughness"].as<double>();
    obs.orientation = map_params["Obstacles"][i]["orientation"].as<double>();

    obs.name = map_params["Obstacles"][i]["name"].as<std::string>();

    obstacle_list.push_back(obs);
  }

  populateMap();
}

bool RandomMapGen::validIndex(grid_map::Index index)
{
  if (index(0) < 0 || index(0) >= grid_map_.getSize()(0) || index(1) < 0 || index(1) >= grid_map_.getSize()(1))
  {
    return false;
  }
  return true;
}

}  // namespace rand_grid_map_gen
