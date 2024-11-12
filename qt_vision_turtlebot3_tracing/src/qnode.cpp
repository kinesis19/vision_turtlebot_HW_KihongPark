#include "../include/qt_vision_turtlebot3_tracing/qnode.hpp"

QNode::QNode() : is_auto_race_active(false)
{
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("qt_vision_turtlebot3_tracing");

  // Initialize Publisher
  publisher_cmd_vel_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

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

    if (is_auto_race_active)  // 주행 활성화 플래그 확인
    {
      runAutoRace();  // 플래그가 true일 때만 실행
    }
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

void QNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image_original = cv_ptr->image;
    image_processed = image_original.clone();

    // Image를 QPixmap으로 변환해서 label에 띄우는 처리
    QImage q_image_original(image_original.data, image_original.cols, image_original.rows, image_original.step, QImage::Format_RGB888);
    QPixmap pixmap_original = QPixmap::fromImage(q_image_original.rgbSwapped());

    emit imageReceivedOriginal(pixmap_original);
  }
  catch (cv_bridge::Exception &e)
  {
    // Debugging: 변환 실패 했을 때
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge Convert Failed: %s", e.what());
  }
}

void QNode::runAutoRace()
{
  // 이미지의 하단에서부터 1/2 영역만 박스 처리하고 감지할 거임
  int image_height = image_processed.rows;
  int image_width = image_processed.cols;
  int roi_height = image_height / 2;  // 하단 1/2 영역의 높이
  cv::Rect roi(0, image_height - roi_height, image_width, roi_height); // 하단 1/2 영역 설정

  // gazebo에서 검출한 카메라의 밝기가 너무 낮아서 밝기하고 대비를 조정함
  cv::Mat brightened_image;
  image_processed.convertTo(brightened_image, -1, 1.5, 50); // alpha=1.5, beta=50

  // main_detect 영역 설정 -> 직진, 우회전, 좌회전 담당하기 위한 영역
  int desired_x = 160; // 원하는 x 위치
  int desired_y = 400; // roi 내에서의 y 위치
  int desired_width = 1600; // 영역의 너비
  int desired_height = 100; // 영역의 높이
  cv::Rect main_detect(desired_x, desired_y, desired_width, desired_height);

  // sub_detect 영역 설정 -> 코너링하기 위한 영역
  int sub_detect_x = 150;
  int sub_detect_y = main_detect.y - desired_height * 2 - 200;
  int sub_detect_width = 1600;
  int sub_detect_height = 100;

  cv::Rect sub_detect(sub_detect_x, sub_detect_y, sub_detect_width, sub_detect_height);

  // 파란색 바운딩 박스 그리기
  cv::rectangle(brightened_image, cv::Rect(roi.x + main_detect.x, roi.y + main_detect.y, main_detect.width, main_detect.height), cv::Scalar(255, 0, 0), 2); // main_detect
  cv::rectangle(brightened_image, cv::Rect(roi.x + sub_detect.x, roi.y + sub_detect.y, sub_detect.width, sub_detect.height), cv::Scalar(0, 0, 255), 2); // sub_detect (빨간색)

  // 하단 1/2 영역을 잘라서 HSV 변환
  cv::Mat cropped_image = brightened_image(roi);
  cv::Mat hsv_image;
  cv::cvtColor(cropped_image, hsv_image, cv::COLOR_BGR2HSV);

  // HSV 범위로 흰색과 노란색 차선 검출
  cv::Mat mask_yellow, mask_white;
  cv::inRange(hsv_image, cv::Scalar(10, 50, 100), cv::Scalar(40, 255, 255), mask_yellow); // 노란색 범위 조정
  cv::inRange(hsv_image, cv::Scalar(0, 0, 200), cv::Scalar(180, 30, 255), mask_white); // 흰색 범위 조정

  // HSV 범위로 빨간색 기믹, 초록색 기믹, 파란색 기믹 검출
  cv::Mat mask_red;
  cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask_red);    // 빨간색 범위 설정 (H 값 낮은 빨간색)
  cv::Mat mask_red2;
  cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), mask_red2); // 높은 H 값 빨간색

  // 두 빨간색 범위 마스크를 결합
  cv::Mat combined_mask_red = mask_red | mask_red2;

  // main_detect 영역에서 빨간색 감지
  cv::Mat main_detect_red = combined_mask_red(main_detect);
  bool red_detected_in_main = cv::countNonZero(main_detect_red) > 0;

  // 빨간색 차선 감지시 멈춤 처리
  if (red_detected_in_main) {
    RCLCPP_WARN(rclcpp::get_logger("QNode"), "완주");
    stopRobot();
  }

  // main_detect와 sub_detect 내의 선 감지
  cv::Mat main_left_white = mask_white(main_detect);
  cv::Mat main_right_yellow = mask_yellow(main_detect);
  cv::Mat sub_left_white = mask_white(sub_detect);
  cv::Mat sub_right_yellow = mask_yellow(sub_detect);

  // countNonZero(): 0이 아닌 픽셀은 255로 -> T/F -> 차선 검출용도로 사용
  bool main_left_white_detected = cv::countNonZero(main_left_white) > 0;
  bool main_right_yellow_detected = cv::countNonZero(main_right_yellow) > 0;
  bool sub_left_white_detected = cv::countNonZero(sub_left_white) > 0;
  bool sub_right_yellow_detected = cv::countNonZero(sub_right_yellow) > 0;

  // 주행 명령 결정
  geometry_msgs::msg::Twist cmd_msg;

  // 직진 조건
  if ((main_left_white_detected && main_right_yellow_detected)) {
    cmd_msg.linear.x = 0.2;
    cmd_msg.angular.z = 0.0;
    RCLCPP_WARN(rclcpp::get_logger("QNode"), "직진! 직진!!");
  } else if (main_right_yellow_detected) {
    cmd_msg.linear.x = 0.05;
    cmd_msg.angular.z = 0.3;
    RCLCPP_WARN(rclcpp::get_logger("QNode"), "좌회전!!");
  } else if (main_left_white_detected) {
    if ((main_left_white_detected && !main_right_yellow_detected) && (sub_left_white_detected && !sub_right_yellow_detected)) {
      cmd_msg.linear.x = 0.2;
      cmd_msg.angular.z = 0.0;
      RCLCPP_WARN(rclcpp::get_logger("QNode"), "아직 직진!!");
    } 
    // else if ((main_left_white_detected && !main_right_yellow_detected) && (!sub_left_white_detected && !sub_right_yellow_detected)) {
    //   cmd_msg.linear.x = 0.2;
    //   cmd_msg.angular.z = 0.0;
    //   RCLCPP_WARN(rclcpp::get_logger("QNode"), "아직 직진!!2222");
    // } else {
    //   cmd_msg.linear.x = 0.01;
    //   cmd_msg.angular.z = -0.5;
    //   RCLCPP_WARN(rclcpp::get_logger("QNode"), "우회전!!");
    // }

    cmd_msg.linear.x = 0.01;
    cmd_msg.angular.z = -0.5;
    RCLCPP_WARN(rclcpp::get_logger("QNode"), "우회전!!");
  } else {
    if(!(main_left_white_detected && main_right_yellow_detected)){
      cmd_msg.linear.x = 0.05;
      cmd_msg.angular.z = -0.3;
      RCLCPP_WARN(rclcpp::get_logger("QNode"), "우회전!!");
    } else {
      cmd_msg.linear.x = 0.01;
      cmd_msg.angular.z = -0.5;
      RCLCPP_WARN(rclcpp::get_logger("QNode"), "라인을 못 찾음");
    }
  }

  // 속도 명령 퍼블리시
  publisher_cmd_vel_->publish(cmd_msg);

  // label로 전송
  QImage q_image_processed(brightened_image.data, brightened_image.cols, brightened_image.rows, brightened_image.step, QImage::Format_RGB888);
  QPixmap pixmap_processed = QPixmap::fromImage(q_image_processed.rgbSwapped());
  emit imageReceivedProcessed(pixmap_processed);
}





void QNode::stopRobot()
{
  geometry_msgs::msg::Twist cmd_msg;
  cmd_msg.linear.x = 0.0;
  cmd_msg.angular.z = 0.0;
  publisher_cmd_vel_->publish(cmd_msg);
}

void QNode::toggleAutoRace()
{
  is_auto_race_active = !is_auto_race_active;  // 토글로 주행 모드 관리함 -> 버튼 하나
  if (!is_auto_race_active) {
    stopRobot();
  }
}