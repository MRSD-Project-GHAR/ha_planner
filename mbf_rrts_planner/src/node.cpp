#include <mbf_rrts_planner/node.hpp>

#include <random>

namespace mbf_rrts_core
{
RRTNode::RRTNode()
{
}

RRTNode::RRTNode(grid_map::GridMap map, int seed)
{
  map_ = map;

  std::default_random_engine generator;
  generator.seed(seed);

  std::uniform_real_distribution<double> x_generator(0, map.getLength()[0]);
  std::uniform_real_distribution<double> y_generator(0, map.getLength()[1]);

  x = x_generator(generator);
  y = y_generator(generator);
}

double RRTNode::getDistance(RRTNodePtr node)
{
  return sqrtf64((x - node->x) * (x - node->x) + (x - node->y) * (x - node->y));
}

double RRTNode::getCost(RRTNodePtr node) {
    return getDistance(node);
}



}  // namespace mbf_rrts_core