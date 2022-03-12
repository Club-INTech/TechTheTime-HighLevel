#pragma once

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "../serial/SerialPort.hpp"
#include "motion_msg_srv/msg/motion.hpp"

using namespace std::chrono_literals;

class MotionPublisher : public rclcpp::Node {
public:

    MotionPublisher(std::chrono::nanoseconds period, std::unique_ptr<SerialPort> gateaway);
    void broadcast_motion(int32_t expected_left_ticks, int32_t expected_right_ticks, int timeout);

private:

    rclcpp::Publisher<motion_msg_srv::msg::Motion>::SharedPtr publisher_;
    std::unique_ptr<SerialPort> microcontroller_gateaway;
    std::chrono::nanoseconds period
}