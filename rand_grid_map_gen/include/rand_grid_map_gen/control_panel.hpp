#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QApplication>
#include <QMainWindow>
#include <QTableView>

#include "rand_grid_map_gen/map_gen.hpp"

/**
 * TODOs
 * */
namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(rand_grid_map_gen::RandomMapGen* map, ros::NodeHandle nh, QWidget* parent = nullptr);
  ~MainWindow();

protected:
  void publishMapButtonPressed();
  void obstacleDropDownChanged();
  void deleteButtonPressed();
  void addButtonPressed();
  void changeButtonPressed();

private:
  Ui::MainWindow* ui;
  rand_grid_map_gen::RandomMapGen* map_;
  ros::NodeHandle nh_;
  ros::Publisher grid_map_publisher_;

  int obstacle_index = 0;

  void changeObstacleFields(rand_grid_map_gen::Obstacle obs);
  void clearObstacleFields();
};

#endif  // MAINWINDOW_H