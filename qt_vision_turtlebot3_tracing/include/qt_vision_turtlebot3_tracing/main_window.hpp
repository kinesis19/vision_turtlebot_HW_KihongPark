#ifndef qt_vision_turtlebot3_tracing_MAIN_WINDOW_H
#define qt_vision_turtlebot3_tracing_MAIN_WINDOW_H

#include "QIcon"
#include "qnode.hpp"
#include "ui_mainwindow.h"
#include <QMainWindow>
#include <QImage>
#include <QPixmap>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();
  QNode* qnode;

private slots:
  void onClickedBtnModeAutoRace();

private:
  Ui::MainWindowDesign* ui;
  cv::Mat image_original;
  cv::Mat mask_yellow;

  void closeEvent(QCloseEvent* event);
};

#endif  // qt_vision_turtlebot3_tracing_MAIN_WINDOW_H
