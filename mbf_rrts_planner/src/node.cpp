#include <mbf_rrts_planner/node.hpp>

#include <random>

namespace mbf_rrts_core
{
RRTNode::RRTNode()
{
}

RRTNode::RRTNode(GridMapPtr map, int seed)
{
  map_ = map;

  std::default_random_engine generator;
  generator.seed(seed);

  std::uniform_real_distribution<double> x_generator(0, map->getLength()[0]);
  std::uniform_real_distribution<double> y_generator(0, map->getLength()[1]);

  x = x_generator(generator);
  y = y_generator(generator);

  while (map->atPosition("elevation", { x, y }) > 1)
  {
    x = x_generator(generator);
    y = y_generator(generator);
  }
}

RRTNode::RRTNode(GridMapPtr map, const geometry_msgs::PoseStamped& pose)
{
  map_ = map;
  x = pose.pose.position.x;
  y = pose.pose.position.y;
}

// TODO: Remove use of redundant getDistance function
double RRTNode::getDistance(RRTNodePtr node)
{
  return sqrtf64((x - node->x) * (x - node->x) + (y - node->y) * (y - node->y));
}

// IMP TODO : Incorporate obstacle passing in the node cost.
double RRTNode::getCost(RRTNodePtr node)
{
  return getDistance(node);
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
}

RRTNode::RRTNodePtr RRTNode::getParent()
{
  return parent_;
}

geometry_msgs::PoseStamped RRTNode::getPoseStampedMsg()
{
  geometry_msgs::PoseStamped node;

  node.pose.position.x = x;
  node.pose.position.x = y;

  return node;
}

}  // namespace mbf_rrts_core