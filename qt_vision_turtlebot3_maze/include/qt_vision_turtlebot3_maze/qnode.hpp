#ifndef qt_vision_turtlebot3_maze_QNODE_HPP_
#define qt_vision_turtlebot3_maze_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif
#include <QThread>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

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
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_scan_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;

  float distance_front_current_;
  float distance_back_current_;
  float distance_left_current_;
  float distance_right_current_;
  float velocity_linear_;
  float velocity_angular_;

  bool isStart;

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);  // 라이다 센서 콜백 메서드

Q_SIGNALS:
  void rosShutDown();
  void updateDistanceLabel(float distFront, float distBack, float distLeft, float distRight);
};

#endif /* qt_vision_turtlebot3_maze_QNODE_HPP_ */
