#pragma once

#include <vector>
#include <list>
#include <memory>
#include <grid_map_core/grid_map_core.hpp>

namespace mbf_rrts_core
{
class RRTNode
{
public:
  typedef std::shared_ptr<RRTNode> RRTNodePtr;

  RRTNode(grid_map::GridMap map, int seed);
  RRTNode();

  double getDistance(RRTNodePtr node);
  double getCost(RRTNodePtr node);

  RRTNodePtr parent;
  std::list<RRTNodePtr> children;

private:
  double x, y;
  grid_map::GridMap map_;
};

}  // namespace mbf_rrts_core
