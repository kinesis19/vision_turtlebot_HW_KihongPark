#include <QApplication>
#include <iostream>

#include "../include/qt_vision_turtlebot3_maze/main_window.hpp"

int main(int argc, char* argv[])
{
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}
