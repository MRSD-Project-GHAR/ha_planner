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
  std::vector<RRTNode::RRTNodePtr> findNearestNeighbours(RRTNode::RRTNodePtr node, double size_of_bounds);


private:

  typedef std::pair<long int, long int> NodeMapCoords;
  typedef std::pair<double, double> GridMapCoords;
  NodeMapCoords getNodeMapCoords(GridMapCoords grid_map_coords);
  GridMapCoords getGridMapCoords(NodeMapCoords node_map_coords);
  NodeMap node_map_;
  GridMapPtr grid_map_;
  int scale_up_factor_;

};

}  // namespace mbf_rrts_core