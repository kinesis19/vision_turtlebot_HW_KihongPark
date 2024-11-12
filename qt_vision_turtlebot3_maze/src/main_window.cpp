#include "../include/qt_vision_turtlebot3_maze/main_window.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();

  // qnode에서 서브스크라이브한 라이다 센서 값 받아오기
  connect(qnode, &QNode::updateDistanceLabel, this, &MainWindow::setLidarSensorDistancetoLabel);

  connect(qnode, &QNode::updateExitNumLabel, this, &MainWindow::setExitNumbertoLabel);

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
  qnode->toggleStart();
  if (qnode->isStart) {
    ui->btnStart->setText("Stop");
  } else {
    ui->btnStart->setText("Start");
  }
}

void MainWindow::setLidarSensorDistancetoLabel(float distFront, float distBack, float distLeft, float distRight)
{
  ui->labelLidarSensorDistFrontNow->setText("Front: " + QString::number(distFront, 'f', 2));
  ui->labelLidarSensorDistBackNow->setText("Back: " + QString::number(distBack, 'f', 2));
  ui->labelLidarSensorDistLeftNow->setText("Left: " + QString::number(distLeft, 'f', 2));
  ui->labelLidarSensorDistRightNow->setText("Right: " + QString::number(distRight, 'f', 2));
}

void MainWindow::setExitNumbertoLabel(int num)
{
  ui->labelExitNum->setText("Exit Num: " + QString::number(num));
}