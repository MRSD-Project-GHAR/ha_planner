#include "mbf_rrts_planner/control_panel.hpp"
#include <nav_msgs/Path.h>

#include "mbf_rrts_planner/ui_control_panel.h"

PlannerController::PlannerController(ros::NodeHandle nh, ros::NodeHandle nh_private, QWidget* parent)
  : QMainWindow(parent), action_client_("move_base", true), ui(new Ui::PlannerController)

{
  ui->setupUi(this);
  map_sub_ = nh_private.subscribe("map_topic", 10, &PlannerController::mapCallback, this);
  start_sub_ = nh_private.subscribe("start_topic", 10, &PlannerController::startPoseCallback, this);
  goal_sub_ = nh_private.subscribe("goal_topic", 10, &PlannerController::goalPoseCallback, this);
  clicked_point_sub_ = nh_private.subscribe("clicked_point", 10, &PlannerController::clickedPointCallback, this);
  path_pub_ = nh_private.advertise<nav_msgs::Path>("path", 10);
  // timer_ = nh_private.createTimer(ros::Duration(0.1), &PlannerController::publishPlan, this);

  plan_service_ = nh_private.advertiseService("generate_plan", &PlannerController::planServiceCallback, this);
  execute_service_ = nh_private.advertiseService("execute_plan", &PlannerController::executeServiceCallback, this);
  get_start_service_ = nh_private.advertiseService("get_start_pose", &PlannerController::getStartServiceCallback, this);
  get_goal_service_ = nh_private.advertiseService("get_goal_pose", &PlannerController::getGoalServiceCallback, this);

  start_.pose.position.x = 0;
  start_.pose.position.y = 1;

  goal_.pose.position.x = 1;
  goal_.pose.position.y = 0;

  ROS_INFO_STREAM("Global Planner: Waiting for move_base action server to start.");
  action_server_initialized_ = action_client_.waitForServer(ros::Duration(5.0));
  if (action_server_initialized_)
  {
    ROS_INFO_STREAM("Global Planner: move_base action server started.");
  }
  else
  {
    ROS_WARN_STREAM("Global Planner: Waiting for action server timed out. Local Planning will be disabled.");
  }

  plan_made_ = false;
  received_map_ = false;

  QTimer* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &PlannerController::timerCallback);
  timer->start(100);
}

void PlannerController::timerCallback() {
  publishPlan();
  ros::spinOnce();
}

PlannerController::~PlannerController()
{
  delete ui;
}

bool PlannerController::planServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  if (received_map_)
  {
    makePlan();
    plan_made_ = true;
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Global Planner: The map has not been received yet. Cannot generate plan.");
    return false;
  }
}

bool PlannerController::executeServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  if (plan_made_ && action_server_initialized_)
  {
    executePlan();
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Global Planner: The plan has not been made yet, or the action server has not been initialized. "
                     "Cannot execute plan");
    return false;
  }
}

bool PlannerController::getStartServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  mode_ = MODE::GETSTARTPOSE;
  ROS_INFO("Global Planner: Waiting for Start Pose");
  return true;
}

bool PlannerController::getGoalServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  mode_ = MODE::GETGOALPOSE;
  ROS_INFO("Global Planner: Waiting for Goal Pose");
  return true;
}

void PlannerController::clickedPointCallback(const geometry_msgs::PointStamped& point)
{
  if (mode_ == MODE::GETGOALPOSE)
  {
    goal_.pose.position = point.point;
    mode_ = MODE::IDLE;
    ROS_INFO("Global Planner: Received Goal pose");
  }
  else if (mode_ == MODE::GETSTARTPOSE)
  {
    start_.pose.position = point.point;
    mode_ = MODE::IDLE;
    ROS_INFO("Global Planner: Received Start pose");
  }
}

void PlannerController::mapCallback(const grid_map_msgs::GridMap& map_msg)
{
  grid_map::GridMapRosConverter::fromMessage(map_msg, map_);
  planner_.setMapPtr(std::make_shared<grid_map::GridMap>(map_));
  received_map_ = true;
}

void PlannerController::startPoseCallback(const geometry_msgs::PoseStamped& start_msg)
{
  start_ = start_msg;
}

void PlannerController::goalPoseCallback(const geometry_msgs::PoseStamped& goal_msg)
{
  goal_ = goal_msg;
}

void PlannerController::makePlan()
{
  std::vector<geometry_msgs::PoseStamped> plan;
  double cost;
  std::string message;
  plan_.clear();

  planner_.setLayerName("traversability");
  planner_.setMapPtr(std::make_shared<grid_map::GridMap>(map_));
  planner_.makePlan(start_, goal_, 0.1, plan_, cost, message);

  for (auto pose : plan_)
  {
    ROS_INFO_STREAM(pose);
  }
}

void PlannerController::executePlan()
{
  for (auto pose : plan_)
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = pose;
    goal.target_pose.header.frame_id = "map";

    ROS_INFO_STREAM("Sending new  ");
    ROS_INFO_STREAM(goal.target_pose);

    action_client_.sendGoal(goal);
    action_client_.waitForResult();
    ROS_INFO_STREAM("Goal reached");
  }
}

void PlannerController::publishPlan()
{
  nav_msgs::Path path;
  static long int seq = 0;
  for (int i = 0; i < plan_.size(); i++)
  {
    path.poses.push_back(plan_[i]);
  }

  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  path.header.seq = seq;
  seq++;

  path_pub_.publish(path);
}

#include "mbf_rrts_planner/moc_control_panel.cpp"