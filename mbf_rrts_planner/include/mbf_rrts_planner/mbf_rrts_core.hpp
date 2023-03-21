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

  void setMapPtr(GridMapPtr grid_map_ptr);

private:
  bool cancel_requested_ = false;
  GridMapPtr grid_map_;
  std::vector<RRTNode::RRTNodePtr> nodes;

  std::vector<RRTNode::RRTNodePtr> findNearestNeighbours(RRTNode::RRTNodePtr node, double threshold);
  void generatePlanFromTree(std::vector<geometry_msgs::PoseStamped>& plan, RRTNode::RRTNodePtr goal);
};
}  // namespace mbf_rrts_core