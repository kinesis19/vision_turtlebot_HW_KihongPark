#include "../include/vision_turtlebot3_tracing/main_control.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MainControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}