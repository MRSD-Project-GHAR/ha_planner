#include "rand_grid_map_gen/map_gen.hpp"
#include <ros/ros.h>
#include "rand_grid_map_gen/control_panel.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "random_map_publisher");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  rand_grid_map_gen::RandomMapGen Map(nh_private);
  int loop_rate;
  nh_private.param("loop_rate", loop_rate, 10);

  ros::Publisher grid_map_publisher = nh_private.advertise<grid_map_msgs::GridMap>("dummy_map", 1000);

  ros::Rate rate(loop_rate);

  QApplication app(argc, argv);
  MainWindow* window = new MainWindow();
  window->show();
  app.exec();

  // while (ros::ok())
  // {
  //   grid_map_publisher.publish(Map.getROSMessage());
  //   ros::spinOnce();
  //   rate.sleep();
  // }

  return 0;
}