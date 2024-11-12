#include "../include/qt_vision_turtlebot3_maze/main_window.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();

  connect(ui->btnStart, &QPushButton::clicked, this, &MainWindow::onClickedBtnStart);

  connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete ui;
}


void MainWindow::onClickedBtnStart()
{
  RCLCPP_WARN(rclcpp::get_logger("QNode"), "버튼 클릭됨");
}