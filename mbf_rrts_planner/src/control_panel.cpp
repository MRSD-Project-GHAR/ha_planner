#include "mbf_rrts_planner/control_panel.hpp"
#include <nav_msgs/Path.h>

#include "mbf_rrts_planner/ui_control_panel.h"

PlannerController::PlannerController(ros::NodeHandle nh, ros::NodeHandle nh_private, QWidget* parent)
  : QMainWindow(parent), action_client_("move_base", true), ui(new Ui::PlannerController)

{
  ui->setupUi(this);
  map_sub_ = nh_private.subscribe("map_topic", 10, &PlannerController::mapCallback, this);
  clicked_point_sub_ = nh_private.subscribe("clicked_point", 10, &PlannerController::clickedPointCallback, this);
  odom_sub_ = nh_private.subscribe("odom", 10, &PlannerController::odomCallback, this);
  path_pub_ = nh_private.advertise<nav_msgs::Path>("path", 10);
  // timer_ = nh_private.createTimer(ros::Duration(0.1), &PlannerController::publishPlan, this);

  // plan_service_ = nh_private.advertiseService("generate_plan", &PlannerController::planServiceCallback, this);
  // execute_service_ = nh_private.advertiseService("execute_plan", &PlannerController::executeServiceCallback, this);
  // get_start_service_ = nh_private.advertiseService("get_start_pose", &PlannerController::getStartServiceCallback,
  // this); get_goal_service_ = nh_private.advertiseService("get_goal_pose", &PlannerController::getGoalServiceCallback,
  // this);
  QObject::connect(ui->generate_plan_button, &QPushButton::clicked, this,
                   &PlannerController::generatePlanButtonClicked);
  QObject::connect(ui->execute_plan_button, &QPushButton::clicked, this, &PlannerController::executePlanButtonClicked);
  QObject::connect(ui->rviz_start_point_button, &QPushButton::clicked, this,
                   &PlannerController::getStartRVizButtonClicked);
  QObject::connect(ui->odom_start_point_button, &QPushButton::clicked, this,
                   &PlannerController::getStartOdomButtonClicked);
  QObject::connect(ui->goal_point_button, &QPushButton::clicked, this, &PlannerController::getGoalButtonClicked);
  QObject::connect(ui->change_iterations_button, &QPushButton::clicked, this,
                   &PlannerController::changeIterationButtonClicked);

  // PUT BUTTON TO GET ODOMETERY FROM TOPIC

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

void PlannerController::timerCallback()
{
  publishPlan();
  if (planning_in_progress_)
  {
    std::stringstream message;
    message << "Generating Plan: Iteration number " << std::to_string(planner_.current_iteration_number);
    ui->generate_plan_label->setText(QString::fromStdString(message.str()));
  }
  else if (planner_thread_.joinable())
  {
    planner_thread_.join();
  }

  ros::spinOnce();
}

PlannerController::~PlannerController()
{
  delete ui;
}

void PlannerController::generatePlanButtonClicked()
{
  if (received_map_)
  {
    // makePlan();
    if (!planner_thread_.joinable())
    {
      planner_thread_ = std::thread(&PlannerController::makePlan, this);
    }
  }
  else
  {
    ROS_ERROR_STREAM("Global Planner: The map has not been received yet. Cannot generate plan.");
  }
}

void PlannerController::executePlanButtonClicked()
{
  if (plan_made_ && action_server_initialized_)
  {
    executePlan();
  }
  else
  {
    ROS_ERROR_STREAM("Global Planner: The plan has not been made yet, or the action server has not been initialized. "
                     "Cannot execute plan");
  }
}

void PlannerController::getStartRVizButtonClicked()
{
  mode_ = MODE::GET_START_POSE_FROM_RVIZ;
  ROS_INFO("Global Planner: Waiting for Start Pose");
  ui->start_point_label->setText(QString::fromStdString("Waiting for Start Pose from RViz"));
}

void PlannerController::getStartOdomButtonClicked()
{
  mode_ = MODE::GET_START_POSE_FROM_ODOM;
  ROS_INFO("Global Planner: Waiting for Start Pose");
  ui->start_point_label->setText(QString::fromStdString("Waiting for Start Pose from Odom"));
}

void PlannerController::getGoalButtonClicked()
{
  mode_ = MODE::GET_GOAL_POSE;
  ROS_INFO("Global Planner: Waiting for Goal Pose");
  ui->goal_point_label->setText(QString::fromStdString("Waiting for Goal Pose from RViz"));
}

void PlannerController::changeIterationButtonClicked()
{
  planner_.setIterations(ui->iteration_numer_spinbox->value());
  std::stringstream message;
  message << "Currently, the number of iterations is " << ui->iteration_numer_spinbox->value();
  ui->iteration_number_label->setText(QString::fromStdString(message.str()));
}

void PlannerController::clickedPointCallback(const geometry_msgs::PointStamped& point)
{
  if (mode_ == MODE::GET_GOAL_POSE)
  {
    goal_.pose.position = point.point;
    mode_ = MODE::IDLE;
    ROS_INFO("Global Planner: Received Goal pose");
    std::stringstream message;
    message << "Goal pose received: (" << std::to_string(goal_.pose.position.x) << ", "
            << std::to_string(goal_.pose.position.y) << ")";
    // message.append(std::to_string(goal_.pose.position.x));
    ui->goal_point_label->setText(QString::fromStdString(message.str()));
  }
  else if (mode_ == MODE::GET_START_POSE_FROM_RVIZ)
  {
    start_.pose.position = point.point;
    mode_ = MODE::IDLE;
    ROS_INFO("Global Planner: Received Start pose from RViz");
    std::stringstream message;
    message << "Start pose received from RViz: (" << std::to_string(start_.pose.position.x) << ", "
            << std::to_string(start_.pose.position.y) << ")";
    // message.append(std::to_string(goal_.pose.position.x));
    ui->start_point_label->setText(QString::fromStdString(message.str()));
  }
}

void PlannerController::mapCallback(const grid_map_msgs::GridMap& map_msg)
{
  grid_map::GridMapRosConverter::fromMessage(map_msg, map_);
  planner_.setMapPtr(std::make_shared<grid_map::GridMap>(map_));
  if (!received_map_)
  {
    ui->generate_plan_label->setText(QString::fromStdString("The map been received, ready to generate the plan."));
  }
  received_map_ = true;
}

void PlannerController::odomCallback(const nav_msgs::Odometry& odom)
{
  if (mode_ == MODE::GET_START_POSE_FROM_ODOM)
  {
    start_.pose = odom.pose.pose;
    mode_ = MODE::IDLE;
    ROS_INFO("Global Planner: Received Start pose from Odometry");
    std::stringstream message;
    message << "Start pose received from odometry: (" << std::to_string(start_.pose.position.x) << ", "
            << std::to_string(start_.pose.position.y) << ")";
    ui->start_point_label->setText(QString::fromStdString(message.str()));
  }
}

void PlannerController::makePlan()
{
  ui->generate_plan_label->setText(QString::fromStdString("Generating Plan: Iteration number 0"));
  planning_in_progress_ = true;
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
  ui->generate_plan_label->setText(QString::fromStdString("Plan Generation Complete."));
  planning_in_progress_ = false;
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