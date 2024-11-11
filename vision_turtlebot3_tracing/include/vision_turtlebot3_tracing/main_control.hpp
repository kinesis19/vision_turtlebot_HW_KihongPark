#ifndef MAIN_CONTROL_HPP
#define MAIN_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"

class MainControl : public rclcpp::Node
{
public:
    MainControl();

    void run(); // Method of Run

private:
};

#endif // MAIN_CONTROL_HPP