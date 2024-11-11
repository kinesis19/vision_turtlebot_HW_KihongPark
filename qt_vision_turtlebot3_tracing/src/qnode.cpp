#include "../include/qt_vision_turtlebot3_tracing/qnode.hpp"

QNode::QNode()
{
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("qt_vision_turtlebot3_tracing");

  // Initialize Subscribe
  subscription_image_ = node->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&QNode::imageCallback, this, std::placeholders::_1));

  this->start();
}

QNode::~QNode()
{
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}

void QNode::run()
{
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}



// Image Callback Method
void QNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat original_image(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()));
  cv::cvtColor(original_image, original_image, cv::COLOR_RGB2BGR);

  // To change QImage and Create QPixmap -> Send Original Image
  QImage qImage(original_image.data, original_image.cols, original_image.rows, original_image.step, QImage::Format_RGB888);
  QPixmap pixmap = QPixmap::fromImage(qImage.rgbSwapped());  // Change to RGB and Setting the QPixmap

  emit imageReceived(pixmap);  // Send the Original Image
}

void QNode::runAutoRace()
{
  
}