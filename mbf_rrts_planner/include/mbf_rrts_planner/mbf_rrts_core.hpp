#pragma once

#include <mbf_abstract_core/abstract_planner.h>
#include <pluginlib/class_list_macros.h>

#include <grid_map_core/grid_map_core.hpp>
#include "mbf_rrts_planner/node.hpp"

namespace mbf_rrts_core
{
/**
 * @class RRTSPlanner
 * @brief This class generates a path through the environment, given the original location of the robot and the goal
 *  point location, using the RRT* Algorithm. This uses a grid_map representation.
 */
class RRTSPlanner : public mbf_abstract_core::AbstractPlanner
{
public:
  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param tolerance If the goal is obstructed, how many meters the planner can relax the constraint
   *        in x and y before failing
   * @param plan The plan... filled by the planner
   * @param cost The cost for the the plan
   * @param message Optional more detailed outcome as a string
   * @return Result code as described on GetPath action result:
   *         SUCCESS         = 0
   *         1..9 are reserved as plugin specific non-error results
   *         FAILURE         = 50  # Unspecified failure, only used for old, non-mfb_core based plugins
   *         CANCELED        = 51
   *         INVALID_START   = 52
   *         INVALID_GOAL    = 53
   *         NO_PATH_FOUND   = 54
   *         PAT_EXCEEDED    = 55
   *         EMPTY_PATH      = 56
   *         TF_ERROR        = 57
   *         NOT_INITIALIZED = 58
   *         INVALID_PLUGIN  = 59
   *         INTERNAL_ERROR  = 60
   *         71..99 are reserved as plugin specific errors
   */

  uint32_t makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                    std::vector<geometry_msgs::PoseStamped>& plan, double& cost, std::string& message);

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time.
   * @remark New on MBF API
   * @return True if a cancel has been successfully requested, false if not implemented.
   */
  bool cancel();

  /**
   * @param grid_map_ptr The pointer to the grid map that will be used for planning
   */
  void setMapPtr(GridMapPtr grid_map_ptr);
  void setLayerName(std::string layer_name);
  void setIterations(int iterations);
  void setNeighbourhoodSize(double neighbour_Size);
  void setSeed(int seed);
  void setDistanceFactor(double distance_factor);

private:
  bool cancel_requested_ = false;
  GridMapPtr grid_map_;
  std::vector<RRTNode::RRTNodePtr> nodes;
  std::string layer_name_;
  int iterations_ = 1000;
  double neighbourhood_size_ = 2.0;
  int seed_ = 10;
  double distance_factor_ = 3.5;

  /**
   * @brief Finds the neighbours nearest to a node in the tree.
   * @details This function finds the neighbours nearest to the given node. The function checks for nodes that are
   * within a certain neighbourhood of the given node. If no such node exists, it just returns the closest node to the
   * given node. Hence, it is always guaranteed to return at least one neighbour if the tree is not empty.
   * @param node The node whose nearest neighbours need to be found
   * @param neighbourhood_size The size of the bounding box within which neighbours have to be found
   * @return A vector of nearest neighbours.
   */
  std::vector<RRTNode::RRTNodePtr> findNearestNeighbours(RRTNode::RRTNodePtr node, double neighbourhood_size);
};
}  // namespace mbf_rrts_core