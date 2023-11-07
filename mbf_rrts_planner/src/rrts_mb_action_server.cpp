#include "mbf_rrts_planner/mbf_rrts_core.hpp"

#include <grid_map_ros/grid_map_ros.hpp>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>

class PlanExecutor
{
public:
  PlanExecutor(ros::NodeHandle nh, ros::NodeHandle nh_private) : action_client_("move_base", true)

  {
    map_sub_ = nh_private.subscribe("map_topic", 10, &PlanExecutor::mapCallback, this);
    start_sub_ = nh_private.subscribe("start_topic", 10, &PlanExecutor::startPoseCallback, this);
    goal_sub_ = nh_private.subscribe("goal_topic", 10, &PlanExecutor::goalPoseCallback, this);
    clicked_point_sub_ = nh_private.subscribe("clicked_point", 10, &PlanExecutor::clickedPointCallback, this);
    path_pub_ = nh_private.advertise<nav_msgs::Path>("path", 10);
    timer_ = nh_private.createTimer(ros::Duration(0.1), &PlanExecutor::publishPlan, this);

    plan_service_ = nh_private.advertiseService("generate_plan", &PlanExecutor::planServiceCallback, this);
    execute_service_ = nh_private.advertiseService("execute_plan", &PlanExecutor::executeServiceCallback, this);
    get_start_service_ = nh_private.advertiseService("get_start_pose", &PlanExecutor::getStartServiceCallback, this);
    get_goal_service_ = nh_private.advertiseService("get_goal_pose", &PlanExecutor::getGoalServiceCallback, this);

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
  }

  bool planServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
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

  bool executeServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
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

  bool getStartServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    mode_ = MODE::GETSTARTPOSE;
    ROS_INFO("Global Planner: Waiting for Start Pose");
    return true;
  }

  bool getGoalServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    mode_ = MODE::GETGOALPOSE;
    ROS_INFO("Global Planner: Waiting for Goal Pose");
    return true;
  }

  void clickedPointCallback(const geometry_msgs::PointStamped& point)
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

  void mapCallback(const grid_map_msgs::GridMap& map_msg)
  {
    grid_map::GridMapRosConverter::fromMessage(map_msg, map_);
    planner_.setMapPtr(std::make_shared<grid_map::GridMap>(map_));
    received_map_ = true;
  }

  void startPoseCallback(const geometry_msgs::PoseStamped& start_msg)
  {
    start_ = start_msg;
  }

  void goalPoseCallback(const geometry_msgs::PoseStamped& goal_msg)
  {
    goal_ = goal_msg;
  }

  void makePlan()
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

  void executePlan()
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

  void publishPlan(const ros::TimerEvent& event)
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
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrts_visualization");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  PlanExecutor plan_executor(nh, nh_private);
  ros::Rate loop_rate(10);

  ros::spin();

  return 0;
}
