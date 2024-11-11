#include "../include/qt_vision_turtlebot3_tracing/main_window.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();

  connect(qnode, &QNode::imageReceived, this, [this](const QPixmap& pixmap) {
      ui->labelCameraImg->setPixmap(pixmap.scaled(ui->labelCameraImg->size(), Qt::KeepAspectRatio)); 
  });
  
  connect(ui->btnModeAutoRace, &QPushButton::clicked, this, &MainWindow::onClickedBtnModeAutoRace);

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete ui;
}


// onCLicked 
void MainWindow::onClickedBtnModeAutoRace()
{
  qnode->runAutoRace();
}
      