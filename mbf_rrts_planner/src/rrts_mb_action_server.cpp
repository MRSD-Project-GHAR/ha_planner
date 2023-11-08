#include "mbf_rrts_planner/control_panel.hpp"

void spinner(const ros::TimerEvent& e) {
  ros::spinOnce();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rrts_visualization");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // PlannerController plan_executor();
  ros::Rate loop_rate(10);

  QApplication app(argc, argv);
  PlannerController* window = new PlannerController(nh, nh_private);
  auto timer = nh.createTimer(ros::Duration(0.1), spinner);
  window->show();
  app.exec();

  return 0;
}
