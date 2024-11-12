#ifndef qt_vision_turtlebot3_tracing_QNODE_HPP_
#define qt_vision_turtlebot3_tracing_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif
#include <QThread>
#include <QImage>
#include <QPixmap>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode();
  ~QNode();

  bool is_auto_race_active;

  void processGaussian();
  void procesHsv();
  void processMask();
  void runAutoRace();
  void stopRobot();
  void toggleAutoRace();

protected:
  void run();

private:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_; // cmd_vel 퍼블리셔 선언
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_;

  cv::Mat image_original;
  cv::Mat image_processed;

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

Q_SIGNALS:
  void imageReceivedOriginal(const QPixmap &pixmap_original);
  void imageReceivedProcessed(const QPixmap &pixmap_processed);
  void rosShutDown();
};

#endif /* qt_vision_turtlebot3_tracing_QNODE_HPP_ */
