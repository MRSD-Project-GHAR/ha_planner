#include <mbf_rrts_planner/node.hpp>

namespace mbf_rrts_core
{
RRTNode::RRTNode()
{
  parent_ = NULL;
}

RRTNode::RRTNode(GridMapPtr map, std::default_random_engine& generator)
{
  parent_ = NULL;
  // std::cout << "Starting formation of node\n";
  map_ = map;
  // std::cout << "Map set!\n";

  // std::cout << "Generator Seeded!\n";

  // std::cout << map->getLength();

  // TODO: hardcoded center of the map
  std::uniform_real_distribution<> x_generator(-map->getLength()[0] / 2.0, map->getLength()[0] / 2.0);
  std::uniform_real_distribution<> y_generator(-map->getLength()[1] / 2.0, map->getLength()[1] / 2.0);

  // std::cout << "Distribution formed!\n";

  x = x_generator(generator);
  y = y_generator(generator);

  auto layers = map->getLayers();
  // std::cout << "layers in this map: " << layers[0];

  while (map->atPosition("elevation", { x, y }) > 1)
  {
    // std::cout << "Oops, was not a good node, remaking the node\n";

    x = x_generator(generator);
    y = y_generator(generator);
  }

  std::cout << "The new node is generated with coordinates " << x << "," << y << "\n";
}

RRTNode::RRTNode(GridMapPtr map, const geometry_msgs::PoseStamped& pose)
{
  parent_ = NULL;
  map_ = map;
  x = pose.pose.position.x;
  y = pose.pose.position.y;
}

// TODO: Remove use of redundant getDistance function
double RRTNode::getDistance(RRTNodePtr node)
{
  return sqrtf64((x - node->x) * (x - node->x) + (y - node->y) * (y - node->y));
}

double RRTNode::getCost(RRTNodePtr node)
{
  bool hits_obstacle = false;
  double total_length = getDistance(node);

  // std::cout << "Distance to this node is " << total_length << "\n";

  double slope = atan2(y - node->y, x - node->x);
  double resolution = map_->getResolution();

  // std::cout << "Original point : " << node->x << ", " << node->y << "\n";
  // std::cout << "New point : " << x << ", " << y << "\n";

  for (double length = 0; length < total_length; length += resolution)
  {
    double new_x = node->x + length * cos(slope);
    double new_y = node->y + length * sin(slope);
    // std::cout << "New interpolated point : " << new_x << ", " << new_y << "\n";

    // Remove hardcode
    if (map_->atPosition("elevation", { new_x, new_y }) > 0.00001)
    {
      hits_obstacle = true;
      break;
    }
  }

  if (hits_obstacle)
    return DBL_MAX;
  else
    return total_length;
}

void RRTNode::setParent(RRTNodePtr parent)
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
  cost = parent_->cost + getCost(parent_);

  std::cout << "Parent Set! Parent coordinates: " << parent_->x << ", " << parent_->y
            << "; Parent cost: " << parent_->cost << "; Child cost: " << cost << "\n";
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

  return node;
}

}  // namespace mbf_rrts_core