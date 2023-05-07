#include "rand_grid_map_gen/control_panel.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include "rand_grid_map_gen/ui_control_panel.h"
#include <yaml-cpp/yaml.h>
#include <fstream>

MainWindow::MainWindow(rand_grid_map_gen::RandomMapGen* map, ros::NodeHandle nh, QWidget* parent)
  : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  map_ = map;
  QObject::connect(ui->publish_map_button, &QPushButton::clicked, this, &MainWindow::publishMapButtonPressed);
  QObject::connect(ui->delete_button, &QPushButton::clicked, this, &MainWindow::deleteButtonPressed);
  QObject::connect(ui->add_button, &QPushButton::clicked, this, &MainWindow::addButtonPressed);
  QObject::connect(ui->change_button, &QPushButton::clicked, this, &MainWindow::changeButtonPressed);
  QObject::connect(ui->save_map_button, &QPushButton::clicked, this, &MainWindow::saveMapButtonPressed);
  QObject::connect(ui->load_map_button, &QPushButton::clicked, this, &MainWindow::loadMapButtonPressed);
  QObject::connect(ui->Robot_a_gen, &QPushButton::clicked, this, &MainWindow::robotAParamButtonPressed);
  QObject::connect(ui->robot_b_gen, &QPushButton::clicked, this, &MainWindow::robotBParamButtonPressed);
  QObject::connect(ui->obstacle_num_dropdown, &QComboBox::currentIndexChanged, this,
                   &MainWindow::obstacleDropDownChanged);

  ui->obstacle_num_dropdown->clear();
  for (int i = 0; i < map_->getNumObstacles(); i++)
  {
    rand_grid_map_gen::Obstacle obs = map_->getObstacle(i);
    ui->obstacle_num_dropdown->addItem(QString::fromStdString(obs.name));
    obstacle_index++;
  }
  obstacle_index++;

  if (map_->getNumObstacles() > 0)
  {
    changeObstacleFields(map_->getObstacle(0));
  }

  ui->x->setValidator(new QDoubleValidator(-50.00, 50.00, 2, this));
  ui->y->setValidator(new QDoubleValidator(-50.00, 50.00, 2, this));

  ui->length->setValidator(new QDoubleValidator(0.0, 50.00, 2, this));
  ui->width->setValidator(new QDoubleValidator(0.0, 50.00, 2, this));
  ui->height->setValidator(new QDoubleValidator(0.0, 50.00, 2, this));

  ui->slope_1->setValidator(new QDoubleValidator(0.0, 89.0, 2, this));
  ui->slope_2->setValidator(new QDoubleValidator(0.0, 89.0, 2, this));
  ui->slope_3->setValidator(new QDoubleValidator(0.0, 89.0, 2, this));
  ui->slope_4->setValidator(new QDoubleValidator(0.0, 89.0, 2, this));

  // ui->orientation->setValidator(new QDoubleValidator(0.0, 360.0, 2, this));

  grid_map_publisher_ = nh.advertise<grid_map_msgs::GridMap>("dummy_map", 1000);
  nh.param("yaml_file_name", yaml_savepath, std::string("/opt/ros/"));
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::publishMapButtonPressed()
{
  grid_map_publisher_.publish(map_->getROSMessage());
}

void MainWindow::saveMapButtonPressed()
{
  map_->saveMap(ui->map_name_textbox->text().toStdString());
}

void MainWindow::loadMapButtonPressed()
{
  map_->loadMap(ui->map_name_textbox->text().toStdString());

  ui->obstacle_num_dropdown->clear();
  obstacle_index = 0;
  for (int i = 0; i < map_->getNumObstacles(); i++)
  {
    rand_grid_map_gen::Obstacle obs = map_->getObstacle(i);
    ui->obstacle_num_dropdown->addItem(QString::fromStdString(obs.name));
    obstacle_index++;
  }
  obstacle_index++;

  if (map_->getNumObstacles() > 0)
  {
    changeObstacleFields(map_->getObstacle(0));
  }
}

void MainWindow::obstacleDropDownChanged()
{
  int index = ui->obstacle_num_dropdown->currentIndex();
  std::string name = ui->obstacle_num_dropdown->itemText(index).toStdString();
  changeObstacleFields(map_->getObstacle(name));
}

void MainWindow::deleteButtonPressed()
{
  int index = ui->obstacle_num_dropdown->currentIndex();
  std::string name = ui->obstacle_num_dropdown->itemText(index).toStdString();
  map_->deleteObstacle(name);

  if (map_->getNumObstacles() > 0)
  {
    changeObstacleFields(map_->getObstacle(0));
    ui->obstacle_num_dropdown->removeItem(index);
    ui->obstacle_num_dropdown->setCurrentIndex(0);
  }
  else
  {
    ui->obstacle_num_dropdown->clear();
  }
}

void MainWindow::addButtonPressed()
{
  rand_grid_map_gen::Obstacle new_obs;

  new_obs.x = ui->x->text().toDouble();
  new_obs.y = ui->y->text().toDouble();

  new_obs.length = ui->length->text().toDouble();
  new_obs.width = ui->width->text().toDouble();
  new_obs.height = ui->height->text().toDouble();

  new_obs.slope1 = ui->slope_1->text().toDouble();
  new_obs.slope2 = ui->slope_2->text().toDouble();
  new_obs.slope3 = ui->slope_3->text().toDouble();
  new_obs.slope4 = ui->slope_4->text().toDouble();

  // new_obs.orientation = ui->orientation->text().toDouble();
  new_obs.orientation = 0;

  // new_obs.roughness = ui->roughness->text().toDouble();
  new_obs.roughness = 0;

  new_obs.name = "Custom Obstacle " + std::to_string(obstacle_index + 10);
  obstacle_index++;

  map_->addObstacle(new_obs);
  ui->obstacle_num_dropdown->addItem(QString::fromStdString(new_obs.name));
  ui->obstacle_num_dropdown->setCurrentText(QString::fromStdString(new_obs.name));
  changeObstacleFields(new_obs);
}

void MainWindow::changeButtonPressed()
{
  rand_grid_map_gen::Obstacle obs;

  obs.x = ui->x->text().toDouble();
  obs.y = ui->y->text().toDouble();

  obs.length = ui->length->text().toDouble();
  obs.width = ui->width->text().toDouble();
  obs.height = ui->height->text().toDouble();

  obs.slope1 = ui->slope_1->text().toDouble();
  obs.slope2 = ui->slope_2->text().toDouble();
  obs.slope3 = ui->slope_3->text().toDouble();
  obs.slope4 = ui->slope_4->text().toDouble();

  // obs.orientation = ui->orientation->text().toDouble();

  // obs.roughness = ui->roughness->text().toDouble();

  obs.name = ui->obstacle_num_dropdown->currentText().toStdString();

  map_->changeObstacle(obs);
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

  // ui->orientation->setText(QString::number(obs.orientation));

  // ui->roughness->setText(QString::number(obs.roughness));
}

void MainWindow::clearObstacleFields()
{
  ui->x->clear();
  ui->y->clear();

  ui->length->clear();
  ui->width->clear();
  ui->height->clear();

  ui->slope_1->clear();
  ui->slope_2->clear();
  ui->slope_3->clear();
  ui->slope_4->clear();

  // ui->orientation->clear();

  // ui->roughness->clear();
}

void MainWindow::robotAParamButtonPressed()
{
  YAML::Node default_params = YAML::LoadFile(yaml_savepath + "filters_chain_default.yaml");
  YAML::Node robot_a_params = default_params;

  double fsoh = ui->fsoh->text().toDouble();
  double mts = ui->mts->text().toDouble();
  double rr = ui->rr->text().toDouble();
  double ef = ui->ef->text().toDouble();

  robot_a_params["grid_map_filters"][2]["params"]["radius"] = rr;
  robot_a_params["grid_map_filters"][4]["params"]["expression"] = "((step(elevation - " + std::to_string(fsoh) +
                                                                  ") + 2) .* "
                                                                  "(-step(original_slope - " +
                                                                  std::to_string(mts) + "*(3.14/180)) + 2)) - 1";

  robot_a_params["grid_map_filters"][7]["params"]["expression"] =
      "(step(elevation) .* (step(elevation - " + std::to_string(fsoh) + "))) + 1";

  robot_a_params["grid_map_filters"][9]["params"]["radius"] = rr;

  robot_a_params["grid_map_filters"][11]["params"]["expression"] = "(slope_factor .* (" + std::to_string(ef) +
                                                                   "*slope + 10000*(step(slope - " +
                                                                   std::to_string(mts) + "*(3.14/180)) + 1)))";

  std::ofstream myfile;
  myfile.open(yaml_savepath + "filters_chain_a.yaml");
  myfile << robot_a_params;
  myfile.close();
}

void MainWindow::robotBParamButtonPressed()
{
  YAML::Node default_params = YAML::LoadFile(yaml_savepath + "filters_chain_default.yaml");
  YAML::Node robot_b_params = default_params;

  double fsoh = ui->fsoh->text().toDouble();
  double mts = ui->mts->text().toDouble();
  double rr = ui->rr->text().toDouble();
  double ef = ui->ef->text().toDouble();

  robot_b_params["grid_map_filters"][2]["params"]["radius"] = rr;
  robot_b_params["grid_map_filters"][4]["params"]["expression"] = "((step(elevation - " + std::to_string(fsoh) +
                                                                  ") + 2) .* "
                                                                  "(-step(original_slope - " +
                                                                  std::to_string(mts) + "*(3.14/180)) + 2)) - 1";

  robot_b_params["grid_map_filters"][7]["params"]["expression"] =
      "(step(elevation) .* (step(elevation - " + std::to_string(fsoh) + "))) + 1";

  robot_b_params["grid_map_filters"][9]["params"]["radius"] = rr;

  robot_b_params["grid_map_filters"][11]["params"]["expression"] = "(slope_factor .* (" + std::to_string(ef) +
                                                                   "*slope + 10000*(step(slope - " +
                                                                   std::to_string(mts) + "*(3.14/180)) + 1)))";

  std::ofstream myfile;
  myfile.open(yaml_savepath + "filters_chain_b.yaml");
  myfile << robot_b_params;
  myfile.close();
}

#include "rand_grid_map_gen/moc_control_panel.cpp"