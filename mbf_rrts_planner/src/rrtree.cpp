#include "mbf_rrts_planner/rrtree.hpp"
#include "CubicSpline.hpp"
#include <tf/transform_datatypes.h>

namespace mbf_rrts_core
{
RRTree::RRTree(GridMapPtr grid_map)
{
  grid_map_ = grid_map;

  // TODO: Remove Hardcoded parameters
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

void RRTree::SmoothPath(std::vector<geometry_msgs::PoseStamped>& plan){
  // The path smoothing can be implemented in two ways:
  // 1) By iteratively sampling points between nodes on the overall path and then checking if
  // two points could be connected to reduce the overall path length

  // This method will not do much because the path that is generated has already been checked for collisions...
  // std::vector<geometry_msgs::PoseStamped> smooth_path;
  // smooth_path.push_back(plan[0]);

  // for (size_t i = 1; i < plan.size() - i; ++i){
  //   geometry_msgs::PoseStamped current_pt = plan[i];
  //   geometry_msgs::PoseStamped next_point = plan[i+1];
  // }

  // 2) Extract waypoints from the path, waypoints will serve as control points for the path-smoothing
  // algorithm. Choose these points at regular intervals along the path
  // for each waypoint, calculate the orientation that the path should have at that point.
  // this can be computed using the direction of the path at that point (tangent vector at each waypoint by looking at the neighboring waypoints)
  // Can use various path-smoothing techniques: Bezier Curvers or polynomial interpolation

  //Sample waypoints along path
  const double waypt_int = 0.1; //intervals at which the waypoints are sampled froom the path
  std::vector<geometry_msgs::PoseStamped> smooth_path;
  std::vector<geometry_msgs::PoseStamped> waypoints;
  std::vector<geometry_msgs::PoseStamped> spline_pt;
   

  //Get waypoints
  for (size_t i=0; i<plan.size(); i++){
    waypoints.push_back(plan[i]);
  
  //Calculate the orientation that the path should have at that point using the tangent vector at each waypoint by looking at the neighboring waypoints
  }
  for (size_t i = 0; i <plan.size(); i++){
    smooth_path.push_back(plan[i]); 
    if (i<plan.size() - 1){
      double x1 = plan[i].pose.position.x, y1 = plan[i].pose.position.y;
      double x2 = plan[i+1].pose.position.x, y2 = plan[i+1].pose.position.y;
      double y_t = y2-y1;
      double x_t = x2-x1;

      
      double yaw = atan2(y_t,x_t);
      smooth_path.back().pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    }

  //Smooth out le curves :) 
  if (plan.size() >= 4){
    // add first waypoint to plan
    smooth_path.push_back(plan[0]);

    for (size_t i =1; i<plan.size() -2, i++){
      CubicSpline spline(plan[i -1], plan[i], plan[i+1], plan[i+2]);
      spline_pt = spline.sampleSpline(0.1);

      for (const auto& point : spline_pt){
        smooth_path.push_back(point)
      }
    }
    smooth_path.push_back(plan[plan.size()-1]);
    plan = smooth_path;
  }
  }
  // plan = smooth_path;
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

  // TODO: Remove hardcode for center of the map
  long int x_id = std::floor((grid_map_coords.first + x_len / 2.0) / resolution);
  long int y_id = std::floor((grid_map_coords.second + y_len / 2.0) / resolution);

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

  for (auto it_ : node_map_[origin_nodemap.first][origin_nodemap.second])
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