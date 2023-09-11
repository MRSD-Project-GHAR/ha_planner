#pragma once

#include <mbf_abstract_nav/abstract_navigation_server.h>
#include <pluginlib/class_loader.hpp>

// Change this to std::unordered_map, once we move to C++11.
#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>

namespace mbf_gridmap_nav
{
/**
 * @brief The CostmapNavigationServer makes Move Base Flex backwards compatible to the old move_base. It combines the
 *        execution classes which use the nav_core/BaseLocalPlanner, nav_core/BaseCostmapPlanner and the
 *        nav_core/RecoveryBehavior base classes as plugin interfaces. These plugin interface are the same for the
 *        old move_base
 */
class GridMapNavigationServer : public mbf_abstract_nav::AbstractNavigationServer
{
public:
  /**
   * @brief Constructor
   * @param tf_listener_ptr Shared pointer to a common TransformListener
   */
  GridMapNavigationServer(const TFPtr& tf_listener_ptr);

  /**
   * @brief Destructor
   */
  virtual ~GridMapNavigationServer();

private:
  /**
   * @brief Loads the plugin associated with the given planner_type parameter.
   * @param planner_type The type of the planner plugin to load.
   * @return true, if the local planner plugin was successfully loaded.
   */
  virtual mbf_abstract_core::AbstractPlanner::Ptr loadPlannerPlugin(const std::string& planner_type);

  /**
   * @brief Initializes the controller plugin with its name and pointer to the costmap
   * @param name The name of the planner
   * @param planner_ptr pointer to the planner object which corresponds to the name param
   * @return true if init succeeded, false otherwise
   */
  virtual bool initializePlannerPlugin(const std::string& name,
                                       const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr);

  /**
   * @brief Loads the plugin associated with the given controller type parameter
   * @param controller_type The type of the controller plugin
   * @return A shared pointer to a new loaded controller, if the controller plugin was loaded successfully,
   *         an empty pointer otherwise.
   */
  virtual mbf_abstract_core::AbstractController::Ptr loadControllerPlugin(const std::string& controller_type);

  /**
   * @brief Initializes the controller plugin with its name, a pointer to the TransformListener
   *        and pointer to the costmap
   * @param name The name of the controller
   * @param controller_ptr pointer to the controller object which corresponds to the name param
   * @return true if init succeeded, false otherwise
   */
  virtual bool initializeControllerPlugin(const std::string& name,
                                          const mbf_abstract_core::AbstractController::Ptr& controller_ptr);

  /**
   * @brief Loads a Recovery plugin associated with given recovery type parameter
   * @param recovery_name The name of the Recovery plugin
   * @return A shared pointer to a Recovery plugin, if the plugin was loaded successfully, an empty pointer otherwise.
   */
  virtual mbf_abstract_core::AbstractRecovery::Ptr loadRecoveryPlugin(const std::string& recovery_type);

  /**
   * @brief Initializes a recovery behavior plugin with its name and pointers to the global and local costmaps
   * @param name The name of the recovery behavior
   * @param behavior_ptr pointer to the recovery behavior object which corresponds to the name param
   * @return true if init succeeded, false otherwise
   */
  virtual bool initializeRecoveryPlugin(const std::string& name,
                                        const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr);

  /**
   * @brief These are the plugin loaders that will be used to load plugins
  */
  pluginlib::ClassLoader<mbf_abstract_core::AbstractRecovery> recovery_plugin_loader_;
  pluginlib::ClassLoader<mbf_abstract_core::AbstractPlanner> planner_plugin_loader_;
  pluginlib::ClassLoader<mbf_abstract_core::AbstractController> controller_plugin_loader_;
};

}  // namespace mbf_gridmap_nav
