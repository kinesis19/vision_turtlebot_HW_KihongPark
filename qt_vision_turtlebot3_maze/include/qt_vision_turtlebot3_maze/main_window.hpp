#ifndef qt_vision_turtlebot3_maze_MAIN_WINDOW_H
#define qt_vision_turtlebot3_maze_MAIN_WINDOW_H

#include <QMainWindow>
#include "QIcon"
#include "qnode.hpp"
#include "ui_mainwindow.h"

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();
  QNode* qnode;

private slots:
  void onClickedBtnStart();

private:
  Ui::MainWindowDesign* ui;
  void closeEvent(QCloseEvent* event);
  void setLidarSensorDistancetoLabel(float distFront, float distBack, float distLeft, float distRight);
  void setExitNumbertoLabel(int num);
};

#endif  // qt_vision_turtlebot3_maze_MAIN_WINDOW_H
