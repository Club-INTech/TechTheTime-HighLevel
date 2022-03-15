#pragma once

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "../serial/SerialPort.hpp"
#include "motion_msg_srv/msg/motion.hpp"
#include "order_codes.hpp"

using namespace std::chrono_literals;

class MotionPublisher : public rclcpp::Node {
public:

    MotionPublisher(std::shared_ptr<scom::SerialPort> gateway);
    void broadcast_motion(int32_t expected_left_ticks, int32_t expected_right_ticks);
    int64_t get_motion_status() const;

private:

    rclcpp::Publisher<motion_msg_srv::msg::Motion>::SharedPtr publisher_;
    std::shared_ptr<scom::SerialPort> microcontroller_gateway;
    MotionStatusCodes motion_status;

};