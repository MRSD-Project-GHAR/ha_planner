#include <mbf_rrts_planner/mbf_rrts_core.hpp>
#include <mbf_rrts_planner/rrtree.hpp>

#include <random>

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
  std::cout << "Planner is making the plan now!\n";
  auto time_now = std::chrono::high_resolution_clock::now();

  if (grid_map_ == NULL)
  {
    return 58;
  }

  int count = 0;

  // TODO: Remove Hardcoded parameters
  int max_iterations = 200;
  int seed = 10;
  double neighbourhood_radius = 2.0;

  std::default_random_engine generator;
  generator.seed(seed);

  // nodes.clear();
  RRTree node_tree(grid_map_);

  RRTNode::RRTNodePtr start_node = std::make_shared<RRTNode>(grid_map_, start);
  start_node->cost = 0;

  // nodes.push_back(start_node);
  node_tree.addNode(start_node);

  RRTNode::RRTNodePtr goal_node = std::make_shared<RRTNode>(grid_map_, goal);
  goal_node->cost = DBL_MAX;
  // std::cout << "Added start and goal node\n";

  for (int i = 0; i < max_iterations; i++)
  {
    // std::cout << "Iteration: " << i << "\n";

    RRTNode::RRTNodePtr random_node = std::make_shared<RRTNode>(grid_map_, generator);
    // std::cout << "Random Node formed\n";

    // std::vector<RRTNode::RRTNodePtr> neighbours = findNearestNeighbours(random_node, neighbourhood_radius);
    std::vector<RRTNode::RRTNodePtr> neighbours = node_tree.findNearestNeighbours(random_node, neighbourhood_radius);
    // std::cout << "Neighbours found. Number of neighbours found = " << neighbours.size() << "\n";

    RRTNode::RRTNodePtr nearest_neighbour = neighbours[0];

    for (auto neighbour : neighbours)
    {
      if (neighbour->getCost(random_node) < nearest_neighbour->getCost(random_node))
      {
        nearest_neighbour = neighbour;
      }
    }
    // std::cout << "Nearest neighbour found. Cost to this neighbour is : " << nearest_neighbour->cost <<"\n";

    if (random_node->getCost(nearest_neighbour) == DBL_MAX)
    {
      continue;
    }
    // std::cout << "This doesn't cross over obstacles\n\n";

    random_node->setParent(nearest_neighbour);
    // nodes.push_back(random_node);
    node_tree.addNode(start_node);

    // std::cout << "This neighbour has now been set as a parent\n";

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
      // std::cout << "Reordered some nodes to make them more efficient\n";
    }

    if ((random_node->cost + random_node->getCost(goal_node)) < goal_node->cost)
    {
      goal_node->setParent(random_node);
    }

    // std::cout << "\n\n";

    // std::cout << "Time taken: " << exec_time.count() << "\n";

    if (cancel_requested_)
    {
      cancel_requested_ = false;
      return 51;
    }
  }
  // std::cout << "RRT tree made. The number of nodes added are : " << nodes.size() << "\n";
  // std::cout << "RRT tree made. Cost is : " << goal_node->cost << "\n";

  if (goal_node->cost == DBL_MAX)
  {
    return 54;
  }
  // std::cout << "Planning Complete. Generating plan from the RRT tree\n";

  // generatePlanFromTree(plan, goal_node);
  node_tree.generatePlanFromTree(plan, goal_node);
  cost = goal_node->cost;
  auto exec_time = std::chrono::high_resolution_clock::now() - time_now;
  std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::microseconds>(exec_time).count() << "\n\n";
  //Takes 5.8 seconds currently

  return 0;
}

std::vector<RRTNode::RRTNodePtr> RRTSPlanner::findNearestNeighbours(RRTNode::RRTNodePtr node, double threshold)
{
  std::vector<RRTNode::RRTNodePtr> nearest_neighbours;
  RRTNode::RRTNodePtr closest_neighbour = nodes[0];

  bool neighbours_found = false;

  for (auto neighbour : nodes)
  {
    if (neighbour->getCost(node) < threshold)
    {
      nearest_neighbours.push_back(neighbour);
      neighbours_found = true;
      // std::cout << "A Neighbour has been found within radius\n";
    }

    if (neighbour->getCost(node) < closest_neighbour->getCost(node))
    {
      closest_neighbour = neighbour;
    }
  }

  if (!neighbours_found)
  {
    nearest_neighbours.push_back(closest_neighbour);
    // std::cout << "No neighbours found within this radius, returning the closest neighbour\n";
  }

  // long int x_start_id = 

  return nearest_neighbours;
}

void RRTSPlanner::generatePlanFromTree(std::vector<geometry_msgs::PoseStamped>& plan, RRTNode::RRTNodePtr goal)
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
