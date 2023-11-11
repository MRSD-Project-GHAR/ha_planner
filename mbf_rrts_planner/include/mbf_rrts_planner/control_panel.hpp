#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <thread>

#include <QApplication>
#include <QMainWindow>
#include <QTableView>
#include <QTimer>

#include <grid_map_ros/grid_map_ros.hpp>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include "mbf_rrts_planner/mbf_rrts_core.hpp"


namespace Ui
{
class PlannerController;
}

class PlannerController : public QMainWindow
{
  Q_OBJECT

public:
  explicit PlannerController(ros::NodeHandle nh, ros::NodeHandle nh_private, QWidget* parent = nullptr);
  ~PlannerController();

  void generatePlanButtonClicked();
  void executePlanButtonClicked();
  void getStartTFButtonClicked();
  void getStartRVizButtonClicked();
  void getGoalButtonClicked();
  void changeIterationButtonClicked();

  void clickedPointCallback(const geometry_msgs::PointStamped& point);

  void mapCallback(const grid_map_msgs::GridMap& map_msg);

  void makePlan();

  void executePlan();

  void publishPlan();

  void timerCallback();

private:
  enum class MODE
  {
    GET_START_POSE_FROM_RVIZ,
    GET_GOAL_POSE,
    IDLE
  };

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;
  grid_map::GridMap map_;
  mbf_rrts_core::RRTSPlanner planner_;
  geometry_msgs::PoseStamped start_;
  geometry_msgs::PoseStamped goal_;

  ros::Subscriber map_sub_;
  ros::Subscriber clicked_point_sub_;
  ros::ServiceServer plan_service_;
  ros::ServiceServer execute_service_;
  ros::ServiceServer get_goal_service_;
  ros::ServiceServer get_start_service_;
  ros::Publisher path_pub_;

  std::vector<geometry_msgs::PoseStamped> plan_;
  bool received_map_;
  bool plan_made_;
  bool action_server_initialized_;
  bool planning_in_progress_ = false;
  bool execution_in_progress_ = false;
  int current_waypoint = 0;
  std::thread planner_thread_;
  std::thread execute_thread_;
  ros::Timer timer_;
  MODE mode_;

  std::string robot_frame_;
  std::string world_frame_;

  Ui::PlannerController* ui;
};

#endif  // MAINWINDOW_H