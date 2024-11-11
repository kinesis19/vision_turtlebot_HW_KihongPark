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

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode();
  ~QNode();

protected:
  void run();

private:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_;

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

Q_SIGNALS:
  void imageReceived(const QPixmap& pixmap);
  void rosShutDown();
};

#endif /* qt_vision_turtlebot3_tracing_QNODE_HPP_ */
