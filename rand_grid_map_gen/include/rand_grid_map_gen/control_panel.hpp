#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QApplication>
#include <QMainWindow>
#include <QTableView>

#include "rand_grid_map_gen/map_gen.hpp"

/**
 * TODOs
 * 1) Display each obstacle's data in the respective fields
 * 2) Implement Delete obstacle
 * 3) Implement Add obstacle
 * 4) Implement Change Obstacle
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

private:
  Ui::MainWindow* ui;
  rand_grid_map_gen::RandomMapGen* map_;
  ros::NodeHandle nh_;
  ros::Publisher grid_map_publisher_;

  void changeObstacleFields(rand_grid_map_gen::Obstacle obs);
};

#endif  // MAINWINDOW_H