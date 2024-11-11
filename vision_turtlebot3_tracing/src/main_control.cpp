#include "../include/vision_turtlebot3_tracing/main_control.hpp"

MainControl::MainControl() : Node("main_control")
{   
    RCLCPP_INFO(this->get_logger(), "MainControl Node 초기화 완료");
}

void MainControl::run()
{
    while (rclcpp::ok())
    {
        
    }
}