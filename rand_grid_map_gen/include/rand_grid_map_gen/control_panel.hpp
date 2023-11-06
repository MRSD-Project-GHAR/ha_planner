#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QApplication>
#include <QMainWindow>
#include <QTableView>

#include <std_srvs/Empty.h>

#include "rand_grid_map_gen/map_gen.hpp"

/**
 * TODOs
 * TODO: Check for bad file names while loading and saving maps
 * */
namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(rand_grid_map_gen::RandomMapGen* map, ros::NodeHandle nh_private, ros::NodeHandle nh, QWidget* parent = nullptr);
  ~MainWindow();

protected:
  void publishMapButtonPressed();
  void obstacleDropDownChanged();
  void deleteButtonPressed();
  void addButtonPressed();
  void changeButtonPressed();
  void saveMapButtonPressed();
  void loadMapButtonPressed();
  void robotAParamButtonPressed();
  void robotBParamButtonPressed();
  void setParamButtonPressed();

private:
  Ui::MainWindow* ui;
  rand_grid_map_gen::RandomMapGen* map_;
  ros::NodeHandle nh_;
  ros::Publisher grid_map_publisher_;
  ros::ServiceClient reload_params_srv_;

  int obstacle_index = 0;

  std::string yaml_savepath;

  void changeObstacleFields(rand_grid_map_gen::Obstacle obs);
  void clearObstacleFields();
};

#endif  // MAINWINDOW_H