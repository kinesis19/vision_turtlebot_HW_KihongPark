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

void QNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    // Convert: ROS Image message to cv::Mat of OpenCV
    cv_bridge::CvImagePtr cv_ptr_1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv_bridge::CvImagePtr cv_ptr_2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image_original = cv_ptr_1->image; // Declare OpenCV cv::Mat
    image_processed = cv_ptr_2->image; // Declare OpenCV cv::Mat

    // Process
    processGaussian();
    procesHsv();
    processMask();

    if (!image_processed.empty())  // 이미지가 비어 있지 않은지 확인
    {
        QImage q_image_original(image_original.data, image_original.cols, image_original.rows, image_original.step, QImage::Format_RGB888);
        QPixmap pixmap_original = QPixmap::fromImage(q_image_original.rgbSwapped());

        QImage q_image_processed(image_processed.data, image_processed.cols, image_processed.rows, image_processed.step, QImage::Format_Grayscale8);
        QPixmap pixmap_processed = QPixmap::fromImage(q_image_processed);

        emit imageReceived(pixmap_original, pixmap_processed);
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "image_processed is empty after processMask");
    }

    // // cv::Mat to QImage(BGR -> RGB)
    // QImage q_image_original(image_original.data, image_original.cols, image_original.rows, image_original.step, QImage::Format_RGB888);
    // QPixmap pixmap_original = QPixmap::fromImage(q_image_original.rgbSwapped()); // BGR to RGB

    // QImage q_image_processed(image_processed.data, image_processed.cols, image_processed.rows, image_processed.step, QImage::Format_RGB888);
    // QPixmap pixmap_processed = QPixmap::fromImage(q_image_processed.rgbSwapped()); // BGR to RGB

    // // Converted QPixmap send to MainWindow
    // emit imageReceived(pixmap_original, pixmap_processed);
  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge Convert Failed: %s", e.what());
  }
}

void QNode::processGaussian()
{
  cv::Mat image_gaussian;
  cv::GaussianBlur(image_processed, image_gaussian, cv::Size(5, 5), 2);
  image_processed = image_gaussian;
}

void QNode::procesHsv()
{
  cv::Mat image_hsv;
  cv::cvtColor(image_processed, image_hsv, cv::COLOR_BGR2HSV);
  image_processed = image_hsv;
}


void QNode::processMask()
{
  cv::Mat mask_yellow;
  cv::Scalar lower_yellow(20, 100, 100);
  cv::Scalar upper_yellow(30, 255, 255);

  // 바이너리 처리
  cv::inRange(image_processed, lower_yellow, upper_yellow, mask_yellow);
  image_processed = mask_yellow;
}


void QNode::runAutoRace()
{
  
}