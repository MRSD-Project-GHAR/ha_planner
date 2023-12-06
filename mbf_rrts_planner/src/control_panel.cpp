#include "mbf_rrts_planner/control_panel.hpp"
#include <nav_msgs/Path.h>

#include "mbf_rrts_planner/ui_control_panel.h"
#include <tf2_ros/transform_listener.h>

PlannerController::PlannerController(ros::NodeHandle nh, ros::NodeHandle nh_private, QWidget* parent)
  : QMainWindow(parent), action_client_("move_base", true), ui(new Ui::PlannerController)

{
  ui->setupUi(this);
  map_sub_ = nh_private.subscribe("map_topic", 10, &PlannerController::mapCallback, this);
  clicked_point_sub_ = nh_private.subscribe("clicked_point", 10, &PlannerController::clickedPointCallback, this);
  path_pub_ = nh_private.advertise<nav_msgs::Path>("path", 10);

  QObject::connect(ui->generate_plan_button, &QPushButton::clicked, this,
                   &PlannerController::generatePlanButtonClicked);
  QObject::connect(ui->execute_plan_button, &QPushButton::clicked, this, &PlannerController::executePlanButtonClicked);
  QObject::connect(ui->rviz_start_point_button, &QPushButton::clicked, this,
                   &PlannerController::getStartRVizButtonClicked);
  QObject::connect(ui->tf_start_point_button, &QPushButton::clicked, this, &PlannerController::getStartTFButtonClicked);
  QObject::connect(ui->goal_point_button, &QPushButton::clicked, this, &PlannerController::getGoalButtonClicked);
  QObject::connect(ui->change_iterations_button, &QPushButton::clicked, this,
                   &PlannerController::changeIterationButtonClicked);

  QObject::connect(ui->cancel_plan_execution_button, &QPushButton::clicked, this,
                   &PlannerController::cancelPlanExecutionButtonClicked);

  nh_private.param<std::string>("robot_frame", robot_frame_, "camera_link");
  nh_private.param<std::string>("world_frame", world_frame_, "world");

  start_.pose.position.x = -4.403289;
  start_.pose.position.y = 4.914849;

  goal_.pose.position.x = 6.215281;
  goal_.pose.position.y = -5.341420;

  ROS_INFO_STREAM("Global Planner: Waiting for move_base action server to start.");
  action_server_initialized_ = action_client_.waitForServer(ros::Duration(5.0));
  if (action_server_initialized_)
  {
    ui->execute_plan_label->setText(QString::fromStdString("Move base action server started, can execute plan after "
                                                           "plan generation"));
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

  if (!execution_in_progress_ && execute_thread_.joinable())
  {
    execute_thread_.join();
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
  if (planning_in_progress_)
  {
    ui->execute_plan_label->setText(QString::fromStdString("The plan is still being generated, cannot execute the "
                                                           "plan."));
  }
  else if (execution_in_progress_)
  {
    ui->execute_plan_label->setText(QString::fromStdString("A plan is already executing."));
  }
  else if (plan_made_ && action_server_initialized_)
  {
    if (!execute_thread_.joinable())
    {
      execute_thread_ = std::thread(&PlannerController::executePlan, this);
    }
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

void PlannerController::getStartTFButtonClicked()
{
  ROS_INFO("Listening to TF for start pose");
  ui->start_point_label->setText(QString::fromStdString("Waiting for Start Pose from TF tree"));

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer.lookupTransform(world_frame_, robot_frame_, ros::Time(0),
                                          ros::Duration(1.0));  // Adjust the timeout as needed
  }
  catch (tf2::TransformException& ex)
  {
    // auto transform = tf_buffer.lookupTransform(robot_frame_, world_frame_, ros::Time(0));
    ui->start_point_label->setText(
        QString::fromStdString("Couldn't get the transform. Error: " + std::string(ex.what())));
    return;
  }

  start_.pose.position.x = transform.transform.translation.x;
  start_.pose.position.y = transform.transform.translation.y;
  std::stringstream message;
  message << "Start pose received from TF Tree: (" << std::to_string(start_.pose.position.x) << ", "
          << std::to_string(start_.pose.position.y) << ")";
  // message.append(std::to_string(goal_.pose.position.x));
  ui->start_point_label->setText(QString::fromStdString(message.str()));
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
  ui->generate_plan_label->setText(
      QString::fromStdString("Plan Generation Complete: Cost of the path is " + std::to_string(cost)));
  plan_made_ = true;
  planning_in_progress_ = false;
}

void PlannerController::executePlan()
{
  execution_in_progress_ = true;
  current_waypoint = 0;
  for (auto pose : plan_)
  {
    ui->execute_plan_label->setText(
        QString::fromStdString("Trying to reach waypoint: " + std::to_string(current_waypoint) + ", pose is " +
                               std::to_string(pose.pose.position.x) + ", " + std::to_string(pose.pose.position.y)));
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = pose;
    goal.target_pose.header.frame_id = "world";

    ROS_INFO_STREAM("Sending new  ");
    ROS_INFO_STREAM(goal.target_pose);

    action_client_.sendGoal(goal);
    // action_client_.waitForResult();

    auto dur = ros::Duration(0.1);

    while (!action_client_.getState().isDone())
    {
      if (cancel_execution_)
      {
        action_client_.cancelAllGoals();
        break;
      }
      dur.sleep();
    }

    if (cancel_execution_)
    {
      ROS_INFO_STREAM("Execution cancelled");
      cancel_execution_ = false;
      break;
    }
    else
    {
      ROS_INFO_STREAM("Goal reached");
      current_waypoint++;
    }
  }
  if (cancel_execution_)
  {
    ui->execute_plan_label->setText(QString::fromStdString("Plan execution cancelled."));
    cancel_execution_ = false;
  }
  else
  {
    ui->execute_plan_label->setText(QString::fromStdString("Reached final waypoint, plan execution complete."));
  }
  execution_in_progress_ = false;
}

void PlannerController::cancelPlanExecutionButtonClicked()
{
  cancel_execution_ = true;
}

void PlannerController::publishPlan()
{
  nav_msgs::Path path;
  static long int seq = 0;
  for (int i = 0; i < plan_.size(); i++)
  {
    path.poses.push_back(plan_[i]);
  }

  path.header.frame_id = "world";
  path.header.stamp = ros::Time::now();
  path.header.seq = seq;
  seq++;

  path_pub_.publish(path);
}

#include "mbf_rrts_planner/moc_control_panel.cpp"