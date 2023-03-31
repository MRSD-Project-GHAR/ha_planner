#include "mbf_rrts_planner/rrtree.hpp"

namespace mbf_rrts_core
{
RRTree::RRTree(GridMapPtr grid_map)
{
  grid_map_ = grid_map;

  scale_up_factor_ = 4;

  double resolution = grid_map_->getResolution();
  auto dimensions = grid_map_->getLength();

  double x_len = dimensions[0];
  double y_len = dimensions[1];

  long int num_cells_x = std::ceil(x_len / (resolution * scale_up_factor_));
  long int num_cells_y = std::ceil(y_len / (resolution * scale_up_factor_));

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
  std::pair<long int, long int> node_map_coords = getNodeMapCoords(std::make_pair(node->x, node->y));
  node_map_[node_map_coords.first][node_map_coords.second].push_back(node);
}

RRTree::NodeMapCoords RRTree::getNodeMapCoords(RRTree::GridMapCoords grid_map_coords)
{
  double resolution = grid_map_->getResolution() * scale_up_factor_;
  auto dimensions = grid_map_->getLength();

  double x_len = dimensions[0];
  double y_len = dimensions[1];
  // std::cout << x_len << ", " << y_len << "erfer\n";

  // TODO: Remove hardcode for center of the map
  long int x_id = std::floor((grid_map_coords.first + x_len / 2.0) / resolution);
  long int y_id = std::floor((grid_map_coords.second + y_len / 2.0) / resolution);
  // std::cout << node->x << ", " << node->y << "ndoe coords\n";
  // std::cout << x_id << ", " << y_id << "Ids individuals\n";

  return std::make_pair(x_id, y_id);
}

RRTree::GridMapCoords RRTree::getGridMapCoords(RRTree::NodeMapCoords node_map_coords)
{
  double resolution = grid_map_->getResolution() * scale_up_factor_;
  auto dimensions = grid_map_->getLength();

  double x_len = dimensions[0];
  double y_len = dimensions[1];

  double x = (node_map_coords.first * resolution) + (resolution / 2.0) - (x_len / 2.0);
  double y = (node_map_coords.second * resolution) + (resolution / 2.0) - (y_len / 2.0);

  return std::make_pair(x, y);
}

std::vector<RRTNode::RRTNodePtr> RRTree::findNearestNeighbours(RRTNode::RRTNodePtr node, double size_of_bounds)
{
  std::vector<RRTNode::RRTNodePtr> neighbours;

  GridMapCoords origin_gridmap = std::make_pair(node->x, node->y);
  NodeMapCoords origin_nodemap = getNodeMapCoords(origin_gridmap);

  for (auto it_ : node_map_[origin_gridmap.first][origin_gridmap.second])
    neighbours.push_back(it_);

  int neighbourhood_size = 1;

  bool within_bounds = true;
  bool node_not_found = (neighbours.size() == 0);

  while (within_bounds || node_not_found)
  {
    int x_left_idx = origin_nodemap.first - neighbourhood_size;
    int x_right_idx = origin_nodemap.first + neighbourhood_size;
    int y_bottom_idx = origin_nodemap.second - neighbourhood_size;
    int y_top_idx = origin_nodemap.second + neighbourhood_size;

    if (x_left_idx < 0)
      x_left_idx = 0;
    if (y_bottom_idx < 0)
      y_bottom_idx = 0;

    if (x_right_idx >= node_map_.size())
      x_right_idx = node_map_.size() - 1;
    if (y_top_idx >= node_map_[0].size())
      y_top_idx = node_map_[0].size() - 1;

    for (int i = x_left_idx; i <= x_right_idx; i++)
    {
      for (auto it_ : node_map_[i][y_top_idx])
        neighbours.push_back(it_);

      for (auto it_ : node_map_[i][y_bottom_idx])
        neighbours.push_back(it_);
    }

    for (int i = y_bottom_idx; i <= y_top_idx; i++)
    {
      for (auto it_ : node_map_[x_right_idx][i])
        neighbours.push_back(it_);

      for (auto it_ : node_map_[x_left_idx][i])
        neighbours.push_back(it_);
    }

    node_not_found = (neighbours.size() == 0);

    neighbourhood_size++;
    double resolution = grid_map_->getResolution() * scale_up_factor_;
    within_bounds = (neighbourhood_size * resolution + (resolution / 2.0)) < size_of_bounds;
  }

  return neighbours;
}

}  // namespace mbf_rrts_core