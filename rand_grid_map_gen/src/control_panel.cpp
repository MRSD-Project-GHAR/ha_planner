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
  QObject::connect(ui->obstacle_num_dropdown, &QComboBox::currentIndexChanged, this, &MainWindow::obstacleDropDownChanged);

  for (int i = 0; i < map_->getNumObstacles(); i++)
  {
    ui->obstacle_num_dropdown->addItem(QString::number(i));
  }

  changeObstacleFields(map->getObstacle(0));

  ui->x->setValidator(new QDoubleValidator(0.0, 10000000000000000.00, 2, this));
  ui->y->setValidator(new QDoubleValidator(0.0, 10000000000000000.00, 2, this));
  
  ui->length->setValidator(new QDoubleValidator(0.0, 10000000000000000.00, 2, this));
  ui->width->setValidator(new QDoubleValidator(0.0, 10000000000000000.00, 2, this));
  ui->height->setValidator(new QDoubleValidator(0.0, 10000000000000000.00, 2, this));
  
  ui->slope_1->setValidator(new QDoubleValidator(0.0, 89.0, 2, this));
  ui->slope_2->setValidator(new QDoubleValidator(0.0, 89.0, 2, this));
  ui->slope_3->setValidator(new QDoubleValidator(0.0, 89.0, 2, this));
  ui->slope_4->setValidator(new QDoubleValidator(0.0, 89.0, 2, this));

  ui->orientation->setValidator(new QDoubleValidator(0.0, 360.0, 2, this));

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

void MainWindow::obstacleDropDownChanged()
{
  int index = ui->obstacle_num_dropdown->currentIndex();
  changeObstacleFields(map_->getObstacle(index));
}

void MainWindow::changeObstacleFields(rand_grid_map_gen::Obstacle obs)
{
  // ui->
  ui->x->setText(QString::number(obs.x));
  ui->y->setText(QString::number(obs.y));

  ui->length->setText(QString::number(obs.length));
  ui->width->setText(QString::number(obs.width));
  ui->height->setText(QString::number(obs.height));

  ui->slope_1->setText(QString::number(obs.slope1));
  ui->slope_2->setText(QString::number(obs.slope2));
  ui->slope_3->setText(QString::number(obs.slope3));
  ui->slope_4->setText(QString::number(obs.slope4));

  ui->orientation->setText(QString::number(obs.orientation));
  
  ui->roughness->setText(QString::number(obs.roughness));
}



#include "rand_grid_map_gen/moc_control_panel.cpp"