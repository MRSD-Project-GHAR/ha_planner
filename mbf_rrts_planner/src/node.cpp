#include <mbf_rrts_planner/node.hpp>
#include <chrono>

namespace mbf_rrts_core
{
RRTNode::RRTNode()
{
  parent_ = NULL;
  layer_name_ = "traversability";
}

RRTNode::RRTNode(GridMapPtr map, std::string layer_name, std::default_random_engine& generator)
{
  parent_ = NULL;
  map_ = map;
  layer_name_ = layer_name;

  // TODO: hardcoded center of the map
  std::uniform_real_distribution<> x_generator(-map->getLength()[0] / 2.0, map->getLength()[0] / 2.0);
  std::uniform_real_distribution<> y_generator(-map->getLength()[1] / 2.0, map->getLength()[1] / 2.0);
  std::uniform_real_distribution<> yaw_generator(0, 2 * M_PI);

  x = x_generator(generator);
  y = y_generator(generator);
  yaw = yaw_generator(generator);

  auto layers = map->getLayers();

  // while (map->atPosition(layer_name_, { x, y }) > -0.5)
  // {
  //   std::cout << "Here " << layer_name_ << "\n";
  //   x = x_generator(generator);
  //   y = y_generator(generator);
  // }

  // std::cout << "The new node is generated with coordinates " << x << "," << y << "\n";
}

RRTNode::RRTNode(GridMapPtr map, std::string layer_name, const geometry_msgs::PoseStamped& pose)
{
  parent_ = NULL;
  map_ = map;
  layer_name_ = layer_name;
  x = pose.pose.position.x;
  y = pose.pose.position.y;
}

double RRTNode::getDistance(RRTNodePtr node)
{
  return sqrtf64((x - node->x) * (x - node->x) + (y - node->y) * (y - node->y));
}

// TODO: Use Bresenham's Line Algorithm for faster speed
// Gets cost for going from 'node' parameter passed to this function to the current object's node
double RRTNode::getCost(RRTNodePtr node, double distance_factor, double yaw_factor)
{
  double total_length = getDistance(node);

  // std::cout << "Distance to this node is " << total_length << "\n";

  double slope = atan2(y - node->y, x - node->x);
  double resolution = map_->getResolution();

  double start_yaw_change = abs(node->yaw - slope);
  start_yaw_change = (start_yaw_change > M_PI) ? (start_yaw_change - M_PI) : start_yaw_change;

  double end_yaw_change = abs (yaw - slope);
  end_yaw_change = (end_yaw_change > M_PI) ? (end_yaw_change - M_PI) : end_yaw_change;

  double total_yaw_change = start_yaw_change + end_yaw_change;
  // std::cout << "Original point : " << node->x << ", " << node->y << "\n";
  // std::cout << "New point : " << x << ", " << y << "\n";

  double cost = distance_factor * total_length + yaw_factor * total_yaw_change;
  for (double length = 0; length < total_length; length += resolution)
  {
    double new_x = node->x + length * cos(slope);
    double new_y = node->y + length * sin(slope);

    cost += map_->atPosition(layer_name_, { new_x, new_y });
  }

  // std::cout << "Cost to this node is " << cost << "\n";
  return cost;
}

void RRTNode::setParent(RRTNodePtr parent, double distance_factor, double yaw_factor)
{
  RRTNodePtr this_node_ptr = shared_from_this();

  // TODO: find an efficient way to do this

  if (parent_)
  {
    auto this_node_it = std::find(parent_->children_.begin(), parent_->children_.end(), this_node_ptr);
    parent_->children_.erase(this_node_it);
  }

  parent_ = parent;

  parent_->children_.push_back(this_node_ptr);
  cost = parent_->cost + getCost(parent_, distance_factor, yaw_factor);

  // std::cout << "Parent Set! Parent: " << parent_->x << ", " << parent_->y << "; Child: " << x << ", " << y
  //           << "; Parent cost: " << parent_->cost << "; Child cost: " << cost << "\n";
}

RRTNode::RRTNodePtr RRTNode::getParent()
{
  return parent_;
}

geometry_msgs::PoseStamped RRTNode::getPoseStampedMsg()
{
  geometry_msgs::PoseStamped node;

  node.pose.position.x = x;
  node.pose.position.y = y;
  node.pose.position.z = 0.5;

  node.pose.orientation.w = 1;

  return node;
}

double RRTNode::quaternionToYaw(geometry_msgs::Quaternion& q)
{
  return atan2(2.0 * (q.z * q.w + q.x * q.y), -1.0 + 2.0 * (q.w * q.w + q.x * q.x));
}

geometry_msgs::Quaternion RRTNode::yawToQuaternion(double yaw)
{
  geometry_msgs::Quaternion q;
  q.x = 0;
  q.y = 0;
  q.z = sin(yaw / 2);
  q.w = cos(yaw / 2);

  return q;
}

}  // namespace mbf_rrts_core