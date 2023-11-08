#ifndef MAINWINDOW_H
#define MAINWINDOW_H

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

  bool planServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool executeServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool getStartServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  bool getGoalServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  void clickedPointCallback(const geometry_msgs::PointStamped& point);

  void mapCallback(const grid_map_msgs::GridMap& map_msg);

  void startPoseCallback(const geometry_msgs::PoseStamped& start_msg);

  void goalPoseCallback(const geometry_msgs::PoseStamped& goal_msg);

  void makePlan();

  void executePlan();

  void publishPlan();

  void timerCallback();

private:
  enum class MODE
  {
    GETSTARTPOSE,
    GETGOALPOSE,
    IDLE
  };

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;
  grid_map::GridMap map_;
  mbf_rrts_core::RRTSPlanner planner_;
  geometry_msgs::PoseStamped start_;
  geometry_msgs::PoseStamped goal_;

  ros::Subscriber start_sub_;
  ros::Subscriber goal_sub_;
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
  ros::Timer timer_;
  MODE mode_;

  Ui::PlannerController* ui;
};

#endif  // MAINWINDOW_H