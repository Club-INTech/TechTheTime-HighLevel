#pragma once

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "../serial/SerialPort.hpp"
#include "motion_msg_srv/msg/motion.hpp"

using namespace std::chrono_literals;

class MotionPublisher : public rclcpp::Node {
public:

    MotionPublisher(std::unique_ptr<scom::SerialPort> gateway);
    void broadcast_motion(int32_t expected_left_ticks, int32_t expected_right_ticks);

private:

    rclcpp::Publisher<motion_msg_srv::msg::Motion>::SharedPtr publisher_;
    std::unique_ptr<scom::SerialPort> microcontroller_gateway;
};