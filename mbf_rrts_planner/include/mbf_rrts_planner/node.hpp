#pragma once

#include <vector>
#include <list>
#include <memory>
#include <grid_map_core/grid_map_core.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <random>

namespace mbf_rrts_core
{
typedef std::shared_ptr<grid_map::GridMap> GridMapPtr;

/**
 * @class RRTNode
 * @brief The Node class that will be used in the tree structure of the RRT* algorithm
 * @details A node in a tree only has one parent, but may have multiple children. Some other functions have been defined
 * that can help with RRT* planning.
 */
class RRTNode : public std::enable_shared_from_this<RRTNode>
{
public:
  typedef std::shared_ptr<RRTNode> RRTNodePtr;

  /**
   * @brief Constructor for constructing a randomly initialized node
   * @param map The pointer to the grid map on which this node is going to be created
   * @param generator The random generator object that will be used to generate the node's parameters. This allows you
   * to pass a seeded generator for repeatability
   */
  RRTNode(GridMapPtr map, std::string layer_name, std::default_random_engine& generator);

  /**
   * @brief Constructor for constructing a Node initialized from a ROS PoseStamped message
   * @param map The pointer to the grid map on which this node is going to be created
   * @param generator The PoseStamped message used to intialize the Node
   */
  RRTNode(GridMapPtr map, std::string layer_name, const geometry_msgs::PoseStamped& pose);

  /**
   * @brief The default constructor, doesn't initialize anything
   */
  RRTNode();

  /**
   * @brief Gives the physical euclidean distance to another node
   * @deprecated For our problem statement, just the distance would not suffice
   * @param node The pointer to the node to which the distance needs to be calculated
   * @return The distance between this node and the passed node parameter
   */
  double getDistance(RRTNodePtr node);

  /**
   * @brief Gives the cost to reach another node. Note: This need not just be the physical distance to another node
   * @param node The pointer to the node to which the cost needs to be calculated
   * @return The cost to traverse between this node and the passed node parameter
   */
  double getCost(RRTNodePtr node, double distance_factor);

  /**
   * @brief Sets the parent of this node.
   * @details A node might not have a parent, or might already have another parent. This function handles both these
   * cases.
   * @param parent The pointer to the node that is going to be the new parent of this node
   */
  void setParent(RRTNodePtr parent, double distance_factor);

  /**
   * @return Returns the pointer to the current parent of this node
   */
  RRTNodePtr getParent();

  /**
   * @return Returns the node data converted to a ROS PoseStamped message
   */
  geometry_msgs::PoseStamped getPoseStampedMsg();
  double cost;

  double x, y;

private:
  RRTNodePtr parent_;
  std::list<RRTNodePtr> children_;
  GridMapPtr map_;
  std::string layer_name_;
};

}  // namespace mbf_rrts_core
