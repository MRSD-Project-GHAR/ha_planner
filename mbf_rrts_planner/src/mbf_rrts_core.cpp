#include <mbf_rrts_planner/mbf_rrts_core.hpp>
#include <mbf_rrts_planner/node.hpp>

// This is what exports this class as ROS plugin. The syntax is PLUGINLIB_EXPORT_CLASS(derived class, base class)
// You need to add some stuff in package.xml and CMakeLists.txt as well. The best way to know what all you have to do to
// export a plugin is to go through this ROS tutorial
// http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin
PLUGINLIB_EXPORT_CLASS(mbf_rrts_core::RRTSPlanner, mbf_abstract_core::AbstractPlanner)

namespace mbf_rrts_core
{
uint32_t RRTSPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                               double tolerance, std::vector<geometry_msgs::PoseStamped>& plan, double& cost,
                               std::string& message)

{
  // std::cout << "Making plan!\n";

  int count = 0;

  // TODO: Remove Hardcoded parameters
  int max_iterations = 10;
  int seed = 100;
  double neighbourhood_radius = 0.5;

  nodes.clear();

  RRTNode::RRTNodePtr start_node = std::make_shared<RRTNode>(grid_map_, start);
  start_node->cost = 0;
  nodes.push_back(start_node);

  RRTNode::RRTNodePtr goal_node = std::make_shared<RRTNode>(grid_map_, goal);
  goal_node->cost = DBL_MAX;

  for (int i = 0; i < max_iterations; i++)
  {
    RRTNode::RRTNodePtr random_node = std::make_shared<RRTNode>(grid_map_, seed);
    std::vector<RRTNode::RRTNodePtr> neighbours = findNearestNeighbours(random_node, neighbourhood_radius);

    RRTNode::RRTNodePtr nearest_neighbour = neighbours[0];

    for (auto neighbour : neighbours)
    {
      if (neighbour->cost < nearest_neighbour->cost)
      {
        nearest_neighbour = neighbour;
      }
    }

    if (random_node->getCost(nearest_neighbour) == DBL_MAX)
    {
      continue;
    }

    random_node->setParent(nearest_neighbour);

    for (auto neighbour : neighbours)
    {
      if (neighbour == nearest_neighbour)
      {
        continue;
      }

      if (neighbour->cost > (random_node->cost + neighbour->getCost(random_node)))
      {
        neighbour->setParent(random_node);
      }
    }

    if (random_node->getCost(goal_node) < goal_node->cost)
    {
      goal_node->setParent(random_node);
    }

    if (cancel_requested_)
    {
      cancel_requested_ = false;
      return 51;
    }
  }

  if (goal_node->cost > tolerance)
  {
    return 54;
  }

  generatePlanFromTree(plan, goal_node);
  cost = goal_node->cost;

  return 0;
}

// IMP TODO

std::vector<RRTNode::RRTNodePtr> RRTSPlanner::findNearestNeighbours(RRTNode::RRTNodePtr node, double threshold)
{
  // RRTNode::RRTNodePtr nearest_neighbour = nodes[0];
  std::vector<RRTNode::RRTNodePtr> nearest_neighbours;

  for (auto neighbour : nodes)
  {
    if (neighbour->cost < threshold)
    {
      nearest_neighbours.push_back(neighbour);
    }
  }

  return nearest_neighbours;
}

// IMP TODO
void RRTSPlanner::generatePlanFromTree(std::vector<geometry_msgs::PoseStamped>& plan, RRTNode::RRTNodePtr goal)
{
  RRTNode::RRTNodePtr current_parent = goal->getParent();
  plan.push_back(goal->getPoseStampedMsg());

  while (current_parent != NULL)
  {
    current_parent = current_parent->getParent();
    plan.push_back(current_parent->getPoseStampedMsg());
  }

  reverse(plan.begin(), plan.end());
}

bool RRTSPlanner::cancel()
{
  std::cout << "Plan is cancelled!";
  cancel_requested_ = true;  // This will allow us to get out of the loop prematurely if we want to
  return true;
}

void RRTSPlanner::setMapPtr(GridMapPtr grid_map)
{
  grid_map_ = grid_map;
}

}  // namespace mbf_rrts_core
