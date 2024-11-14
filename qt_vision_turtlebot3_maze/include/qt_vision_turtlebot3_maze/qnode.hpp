#ifndef qt_vision_turtlebot3_maze_QNODE_HPP_
#define qt_vision_turtlebot3_maze_QNODE_HPP_

#include <cstdint>
#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif
#include <QThread>
#include <cmath>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/int32.hpp"

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode();
  ~QNode();

  bool isStart;

  void toggleStart();

protected:
  void run();

private:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_scan_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_exit_num_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;

  float distance_front_current_;
  float distance_back_current_;
  float distance_left_current_;
  float distance_right_current_;
  float velocity_linear_;
  float velocity_angular_;

  float distance_front_avg_;
  float distance_back_avg_;
  float distance_left_avg_;
  float distance_right_avg_;

  bool isNeartoWall;
  bool isFollowLeftWall;

  int32_t exitNum;
  float aryLidarSensorValue[4];

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);  // 라이다 센서 콜백 메서드
  void exitNumCallback(const std_msgs::msg::Int32::SharedPtr num);
  void stopRobot();
  void runEscape();

  void checkWallNear(); // 충돌 방지 메서드

  int getHighValuefromLiDARSensor();
  double getCurrentAngle();

Q_SIGNALS:
  void rosShutDown();
  void updateDistanceLabel(float distFront, float distBack, float distLeft, float distRight);
  void updateExitNumLabel(int exitNum);
};

#endif /* qt_vision_turtlebot3_maze_QNODE_HPP_ */
