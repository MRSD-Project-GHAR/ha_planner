#include "mbf_rrts_planner/rrtree.hpp"

namespace mbf_rrts_core
{
RRTree::RRTree(GridMapPtr grid_map)
{
  grid_map_ = grid_map;

  scale_down_factor_ = 4;

  double resolution = grid_map_->getResolution();
  auto dimensions = grid_map_->getLength();

  double x_len = dimensions[0];
  double y_len = dimensions[1];

  long int num_cells_x = std::ceil(x_len / (resolution * scale_down_factor_));
  long int num_cells_y = std::ceil(y_len / (resolution * scale_down_factor_));

  std::list<RRTNode::RRTNodePtr> empty_node_list;
  std::vector<std::list<RRTNode::RRTNodePtr>> empty_row(num_cells_y + 1);

  node_map_ = NodeMap(num_cells_x + 1, empty_row);
}

void RRTree::generatePlanFromTree(std::vector<geometry_msgs::PoseStamped>& plan, RRTNode::RRTNodePtr goal)
{
  plan.push_back(goal->getPoseStampedMsg());
  RRTNode::RRTNodePtr current_parent = goal->getParent();

  // std::cout << "Back tracking from goal node to start node to get the path.\n";
  while (current_parent != NULL)
  {
    plan.push_back(current_parent->getPoseStampedMsg());
    current_parent = current_parent->getParent();
  }

  // std::cout << "Plan made with " << plan.size() << " nodes\n";

  reverse(plan.begin(), plan.end());
}

void RRTree::addNode(RRTNode::RRTNodePtr node)
{
  auto node_map_coords = getNodeMapCoords(node);
  long int node_map_x = node_map_coords.first;
  long int node_map_y = node_map_coords.second;

  node_map_[node_map_x][node_map_y].push_back(node);
}

std::pair<long int, long int> RRTree::getNodeMapCoords(RRTNode::RRTNodePtr node)
{
  double resolution = grid_map_->getResolution();
  auto dimensions = grid_map_->getLength();

  double x_len = dimensions[0];
  double y_len = dimensions[1];
  // std::cout << x_len << ", " << y_len << "erfer\n";

  // TODO: Remove hardcode for center of the map
  long int x_id = std::floor((node->x + x_len / 2.0) / resolution);
  long int y_id = std::floor((node->y + y_len / 2.0) / resolution);
  // std::cout << node->x << ", " << node->y << "ndoe coords\n";
  // std::cout << x_id << ", " << y_id << "Ids individuals\n";

  return std::make_pair(x_id, y_id);
}

}  // namespace mbf_rrts_core