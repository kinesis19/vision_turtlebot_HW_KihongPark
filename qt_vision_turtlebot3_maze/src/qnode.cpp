#include "../include/qt_vision_turtlebot3_maze/qnode.hpp"

QNode::QNode() : distance_front_current_(0.0), distance_back_current_(0.0), distance_left_current_(0.0), distance_right_current_(0.0), velocity_linear_(0.0), velocity_angular_(0.0), isStart(false), exitNum(0), isNeartoWall(false)
{
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("qt_vision_turtlebot3_maze");

  // 서브스크라이버 초기화: /scan 토픽 
  subscriber_scan_ = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&QNode::scanCallback, this, std::placeholders::_1));
  // /turtlebot3_status_manager/number 토픽
  subscriber_exit_num_ = node->create_subscription<std_msgs::msg::Int32>("/turtlebot3_status_manager/number", 10, std::bind(&QNode::exitNumCallback, this, std::placeholders::_1));

  // 퍼블리셔 초기화: /cmd_vel 토픽
  publisher_cmd_vel_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // 배열 초기화
  for (int i = 0; i < 4; i++) 
  {
    aryLidarSensorValue[i] = 0;
  }

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
  // rclcpp::WallRate loop_rate(20);
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

  aryLidarSensorValue[0] = distance_front_current_;
  aryLidarSensorValue[1] = distance_back_current_;
  aryLidarSensorValue[2] = distance_left_current_;
  aryLidarSensorValue[3] = distance_right_current_;

  emit updateDistanceLabel(distance_front_current_, distance_back_current_, distance_left_current_, distance_right_current_);
}

void QNode::exitNumCallback(const std_msgs::msg::Int32::SharedPtr num)
{
  exitNum = num->data;

  emit updateExitNumLabel(exitNum);
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


/* memo:
* 미로 탈출 알고리즘 구현하기 전에, 먼저 생각을 해보자.
* 에이스타(A*) 알고리즘은 적용하지 않겠음. 우선은 Exit Num1로 탈출하는 알고리즘부터 설계하곘음.
* 여러가직 방법이 있는데, 네 방향(front, back, left, right)의 LiDAR 센서 값 받아서, 가장 큰 값(= 물체와 멀리 있는) 방향으로 이동하도록 하는 거임.
* 이렇게 해서 먼저 구현해 보도록 하겠음.
*/
// 미로 탈출 메서드
void QNode::runEscape()
{

  geometry_msgs::msg::Twist cmd_msg;

  float speedX = 0.05;
  // 직진 처리
  if (distance_front_current_ > 0.2) {
    // 직진 도중 오른쪽에 벽이 없을 때
    if (distance_right_current_ > 0.25) {
      // 벽이 없지만, 값이 최댓값(3.5)을 넘어설 때: 첫 번째 코너에 대한 예외처리
      if (distance_right_current_ > 3.5) {
        cmd_msg.linear.x = speedX;
        cmd_msg.angular.z = 0.0;
        RCLCPP_WARN(rclcpp::get_logger("QNode"), "우! 회! 전! 예외처리11111");
      } else {
        // 벽이 없고, 왼쪽 벽과의 거리가 가까울 때: 왼쪽 벽 트래킹
        if (distance_left_current_ < 0.18) {
          cmd_msg.linear.x = speedX;
          cmd_msg.angular.z = 0.0;
          RCLCPP_WARN(rclcpp::get_logger("QNode"), "우! 회! 전! 예외처리22222");
        } else {
          cmd_msg.linear.x = speedX;
          cmd_msg.angular.z = -0.5;
          RCLCPP_WARN(rclcpp::get_logger("QNode"), "우! 회! 전!");
        }
      }
    } else if (distance_right_current_ > 0.25) {
      cmd_msg.linear.x = speedX;
      cmd_msg.angular.z = 0.5;
      RCLCPP_WARN(rclcpp::get_logger("QNode"), "좌! 회! 전!");
    } else {
      cmd_msg.linear.x = 0.2;
      cmd_msg.angular.z = 0.0;
      RCLCPP_WARN(rclcpp::get_logger("QNode"), "직! 진!");
    }
  } else {
    RCLCPP_WARN(rclcpp::get_logger("QNode"), "멈! 춤!");
  }


  publisher_cmd_vel_->publish(cmd_msg);
}