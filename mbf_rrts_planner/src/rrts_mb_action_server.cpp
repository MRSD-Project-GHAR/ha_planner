#include "mbf_rrts_planner/mbf_rrts_core.hpp"

#include <grid_map_ros/grid_map_ros.hpp>

#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>

class PlanExecutor
{
public:
  PlanExecutor(ros::NodeHandle nh, ros::NodeHandle nh_private) : action_client_("move_base", true)

  {
    ROS_INFO_STREAM("Waiting for move_base action server to start.");
    action_client_.waitForServer();
    ROS_INFO_STREAM("move_base action server started.");
    map_sub_ = nh.subscribe("map_topic", 10, &PlanExecutor::mapCallback, this);
    start_sub_ = nh.subscribe("start_topic", 10, &PlanExecutor::startPoseCallback, this);
    goal_sub_ = nh.subscribe("goal_topic", 10, &PlanExecutor::goalPoseCallback, this);
    plan_pub_ = nh.advertise<nav_msgs::Path>("planned_path", 10);
    plan_service_ = nh.advertiseService("generate_plan", &PlanExecutor::planServiceCallback, this);
    execute_service_ = nh.advertiseService("execute_plan", &PlanExecutor::executeServiceCallback, this);

    start_.pose.position.x = 0;
    start_.pose.position.y = 1;

    goal_.pose.position.x = 1;
    goal_.pose.position.y = 0;

    plan_made_ = false;
    plan_.header.frame_id = "odom";
    plan_.header.seq = 0;
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
      return false;
    }
  }

  bool executeServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    if (plan_made_)
    {
      executePlan();
      return true;
    }
    else
    {
      return false;
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
    plan_.poses.clear();

    planner_.setLayerName("traversability");
    planner_.setMapPtr(std::make_shared<grid_map::GridMap>(map_));
    planner_.makePlan(start_, goal_, 0.1, plan_.poses, cost, message);

    for (auto pose : plan_.poses)
    {
      ROS_INFO_STREAM(pose);
    }
  }

  void executePlan()
  {
    for (auto pose : plan_.poses)
    {
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose = pose;
      goal.target_pose.header.frame_id = "odom";
      
      ROS_INFO_STREAM("Sending new  ");
      ROS_INFO_STREAM(goal.target_pose);

      action_client_.sendGoal(goal);
      action_client_.waitForResult();
      ROS_INFO_STREAM("Goal reached");
    }
  }

  void pubPlan() {
    plan_.header.seq++;
    plan_.header.stamp = ros::Time::now();
    plan_pub_.publish(plan_);
  }

private:
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;
  grid_map::GridMap map_;
  mbf_rrts_core::RRTSPlanner planner_;
  geometry_msgs::PoseStamped start_;
  geometry_msgs::PoseStamped goal_;

  ros::Subscriber start_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber map_sub_;
  ros::Publisher plan_pub_;
  ros::ServiceServer plan_service_;
  ros::ServiceServer execute_service_;

  // std::vector<geometry_msgs::PoseStamped> plan_;
  nav_msgs::Path plan_;

  bool received_map_;
  bool plan_made_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrts_visualization");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  PlanExecutor plan_executor(nh, nh_private);
  ros::Rate loop_rate(10);

  // ros::spin();
  while (ros::ok())
  {
    plan_executor.pubPlan();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
