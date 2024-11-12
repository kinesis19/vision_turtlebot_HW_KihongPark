#include "../include/qt_vision_turtlebot3_maze/qnode.hpp"

QNode::QNode() : distance_front_current_(0.0), distance_back_current_(0.0), distance_left_current_(0.0), distance_right_current_(0.0), velocity_linear_(0.0), velocity_angular_(0.0), isStart(false)
{
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("qt_vision_turtlebot3_maze");

  // 서브스크라이버 초기화: /scan 토픽 
  subscriber_scan_ = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&QNode::scanCallback, this, std::placeholders::_1));
  // 퍼블리셔 초기화: /cmd_vel 토픽
  publisher_cmd_vel_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

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
    if (isStart)
    {
      runEscape();
    }
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

// /scan 토픽 콜백 메서드
void QNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  distance_front_current_ = msg->ranges[0];
  distance_back_current_= msg->ranges[msg->ranges.size() / 2];
  distance_left_current_= msg->ranges[msg->ranges.size() / 4];
  distance_right_current_ = msg->ranges[msg->ranges.size() * 3 / 4];

  emit updateDistanceLabel(distance_front_current_, distance_back_current_, distance_left_current_, distance_right_current_);
}

// Turtlebot3 정지 메서드
void QNode::stopRobot()
{
  geometry_msgs::msg::Twist cmd_msg;
  cmd_msg.linear.x = 0.0;
  cmd_msg.angular.z = 0.0;
  publisher_cmd_vel_->publish(cmd_msg);

  RCLCPP_WARN(rclcpp::get_logger("QNode"), "Turtlebot3 작동 정지");
}

// Turtlebot3 미로 찾기 플래그 설정을 위한 토글 메서드
void QNode::toggleStart()
{
  isStart = !isStart;
  if (!isStart) {
    stopRobot();
  } else {
    RCLCPP_WARN(rclcpp::get_logger("QNode"), "Turtlebot3 작동 시작");
  }
}

void QNode::runEscape()
{
  RCLCPP_WARN(rclcpp::get_logger("QNode"), "미로 탈출 로직 실행 중..");
}