#include "../include/vision_turtlebot3_tracing/main_control.hpp"

MainControl::MainControl() : Node("main_control")
{   
    // Initialize Publisher
    publisher_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "MainControl Node Initialize Done!");

    run();
}

// Main Process
void MainControl::run()
{
    while (rclcpp::ok())
    {
        // Debugging: Check cmd_vel of Turtlebot3
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.1;
        std::cout << "x: " << twist_msg.linear.x << " z: " << twist_msg.angular.z << std::endl;

        publisher_cmd_vel_->publish(twist_msg);
    }
}
