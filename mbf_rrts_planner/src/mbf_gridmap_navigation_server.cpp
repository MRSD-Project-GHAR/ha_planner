#include "mbf_rrts_planner/gridmap_navigation_server.hpp"
#include <iostream>
#include <string>

namespace mbf_gridmap_nav
{
// Initialize the base class constructor and all the plugin loaders
GridMapNavigationServer::GridMapNavigationServer(const TFPtr& tf_listener_ptr)
  : mbf_abstract_nav::AbstractNavigationServer(tf_listener_ptr)
  , planner_plugin_loader_("mbf_abstract_core", "mbf_abstract_core::AbstractPlanner")
  , recovery_plugin_loader_("mbf_abstract_core", "mbf_abstract_core::AbstractRecovery")
  , controller_plugin_loader_("mbf_abstract_core", "mbf_abstract_core::AbstractController")

{
  // For more info on why we are calling these functions, head over to our website
  // https://mrsdprojects.ri.cmu.edu/2023teami/move-base-flex-navigation-server/
  initializeServerComponents();

  startActionServers();
}

GridMapNavigationServer::~GridMapNavigationServer()
{
  // This code is to ensure that all plugins and action servers shutdown properly before the navigation server destructs
  planner_plugin_manager_.clearPlugins();
  controller_plugin_manager_.clearPlugins();
  recovery_plugin_manager_.clearPlugins();

  action_server_recovery_ptr_.reset();
  action_server_exe_path_ptr_.reset();
  action_server_get_path_ptr_.reset();
  action_server_move_base_ptr_.reset();
}

mbf_abstract_core::AbstractPlanner::Ptr GridMapNavigationServer::loadPlannerPlugin(const std::string& planner_type)
{
  mbf_abstract_core::AbstractPlanner::Ptr planner_ptr = NULL;

  // This block is what loads the planner class and gives it to the plugin manager. For more info, consult the website
  // https://mrsdprojects.ri.cmu.edu/2023teami/move-base-flex-navigation-server/
  try
  {
    planner_ptr = boost::static_pointer_cast<mbf_abstract_core::AbstractPlanner>(
        planner_plugin_loader_.createInstance(planner_type));

    std::string planner_name = planner_plugin_loader_.getName(planner_type);
    ROS_DEBUG_STREAM("mbf_costmap_core-based planner plugin " << planner_name << " loaded.");
  }
  catch (const pluginlib::PluginlibException& ex_mbf_core)
  {
    ROS_DEBUG_STREAM("Failed to load the " << planner_type << " planner." << ex_mbf_core.what());
  }

  return planner_ptr;
}

bool GridMapNavigationServer::initializePlannerPlugin(const std::string& name,
                                                      const mbf_abstract_core::AbstractPlanner::Ptr& planner_ptr)
{
  std::cout << "Dummy Planner plugin initialized\n\n";
  return true;
}

mbf_abstract_core::AbstractController::Ptr
GridMapNavigationServer::loadControllerPlugin(const std::string& controller_type)
{
  mbf_abstract_core::AbstractController::Ptr controller_ptr = NULL;

  // This block is what loads the controller class and gives it to the plugin manager. For more info, consult the
  // website https://mrsdprojects.ri.cmu.edu/2023teami/move-base-flex-navigation-server/
  try
  {
    controller_ptr = boost::static_pointer_cast<mbf_abstract_core::AbstractController>(
        controller_plugin_loader_.createInstance(controller_type));

    std::string controller_name = controller_plugin_loader_.getName(controller_type);
    ROS_DEBUG_STREAM("mbf_costmap_core-based controller plugin " << controller_name << " loaded.");
  }
  catch (const pluginlib::PluginlibException& ex_mbf_core)
  {
    ROS_DEBUG_STREAM("Failed to load the " << controller_type << " controller." << ex_mbf_core.what());
  }

  return controller_ptr;
}

bool GridMapNavigationServer::initializeControllerPlugin(
    const std::string& name, const mbf_abstract_core::AbstractController::Ptr& controller_ptr)
{
  std::cout << "Dummy Controller plugin initialized\n\n";
  return true;
}

mbf_abstract_core::AbstractRecovery::Ptr GridMapNavigationServer::loadRecoveryPlugin(const std::string& recovery_type)
{
  mbf_abstract_core::AbstractRecovery::Ptr recovery_ptr = NULL;

  // This block is what loads the recovery class and gives it to the plugin manager. For more info, consult the website
  // https://mrsdprojects.ri.cmu.edu/2023teami/move-base-flex-navigation-server/
  try
  {
    recovery_ptr = boost::static_pointer_cast<mbf_abstract_core::AbstractRecovery>(
        recovery_plugin_loader_.createInstance(recovery_type));

    std::string recovery_name = recovery_plugin_loader_.getName(recovery_type);
    ROS_DEBUG_STREAM("mbf_costmap_core-based recovery plugin " << recovery_name << " loaded.");
  }
  catch (const pluginlib::PluginlibException& ex_mbf_core)
  {
    ROS_DEBUG_STREAM("Failed to load the " << recovery_type << " recovery." << ex_mbf_core.what());
  }

  return recovery_ptr;
}

bool GridMapNavigationServer::initializeRecoveryPlugin(const std::string& name,
                                                       const mbf_abstract_core::AbstractRecovery::Ptr& behavior_ptr)
{
  std::cout << "Dummy Recovery plugin initialized\n\n";
  return true;
}

}  // namespace mbf_gridmap_nav