#include "../include/qt_vision_turtlebot3_tracing/main_window.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();
  
  connect(qnode, &QNode::imageReceived, this, [this](const QPixmap &pixmap_original, const QPixmap& pixmap_processed) {
    ui->labelCameraImg_1->setPixmap(pixmap_original.scaled(ui->labelCameraImg_1->size(), Qt::KeepAspectRatio));
    ui->labelCameraImg_2->setPixmap(pixmap_processed.scaled(ui->labelCameraImg_2->size(), Qt::KeepAspectRatio));
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
