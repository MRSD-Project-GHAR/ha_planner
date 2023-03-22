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

class RRTNode : public std::enable_shared_from_this<RRTNode>
{
public:
  typedef std::shared_ptr<RRTNode> RRTNodePtr;

  RRTNode(GridMapPtr map, std::default_random_engine& generator);
  RRTNode(GridMapPtr map, const geometry_msgs::PoseStamped& pose);
  RRTNode();

  double getDistance(RRTNodePtr node);
  double getCost(RRTNodePtr node);
  void setParent(RRTNodePtr parent);
  RRTNodePtr getParent();
  geometry_msgs::PoseStamped getPoseStampedMsg();
  double cost;

private:
  RRTNodePtr parent_;
  std::list<RRTNodePtr> children_;
  double x, y;
  GridMapPtr map_;
};

}  // namespace mbf_rrts_core
