#pragma once

#include "mbf_rrts_planner/node.hpp"

namespace mbf_rrts_core
{
class RRTree
{
public:
  typedef std::vector<std::vector<std::list<RRTNode::RRTNodePtr>>> NodeMap;
  RRTree(GridMapPtr grid_map);
  void generatePlanFromTree(std::vector<geometry_msgs::PoseStamped>& plan, RRTNode::RRTNodePtr goal);
  void addNode(RRTNode::RRTNodePtr node);

private:

  std::pair<long int, long int> getNodeMapCoords(RRTNode::RRTNodePtr);
  NodeMap node_map_;
  GridMapPtr grid_map_;
  int scale_down_factor_;

};

}  // namespace mbf_rrts_core