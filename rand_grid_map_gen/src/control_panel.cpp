#include "rand_grid_map_gen/control_panel.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include "rand_grid_map_gen/ui_control_panel.h"

MainWindow::MainWindow(rand_grid_map_gen::RandomMapGen* map, ros::NodeHandle nh, QWidget* parent)
  : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  map_ = map;
  QObject::connect(ui->publish_map_button, &QPushButton::clicked, this, &MainWindow::publishMapButtonPressed);

  // ui->obstacle_num_dropdown->addItems();

  for (int i = 0; i < map_->getNumObstacles(); i++) {
    ui->obstacle_num_dropdown->addItem(QString::number(i));
  }

  grid_map_publisher_ = nh.advertise<grid_map_msgs::GridMap>("dummy_map", 1000);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::publishMapButtonPressed()
{
  grid_map_publisher_.publish(map_->getROSMessage());
}

#include "rand_grid_map_gen/moc_control_panel.cpp"