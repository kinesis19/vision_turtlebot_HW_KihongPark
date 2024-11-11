#ifndef qt_vision_turtlebot3_tracing_MAIN_WINDOW_H
#define qt_vision_turtlebot3_tracing_MAIN_WINDOW_H

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

private:
  Ui::MainWindowDesign* ui;
  void closeEvent(QCloseEvent* event);
};

#endif  // qt_vision_turtlebot3_tracing_MAIN_WINDOW_H
