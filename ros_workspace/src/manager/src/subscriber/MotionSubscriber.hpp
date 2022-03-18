#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "motion_msg_srv/msg/motion.hpp"

using std::placeholders::_1;

class MotionSubscriber : public rclcpp::Node {
public:
    MotionSubscriber();

private:
    void motion_callback(const motion_msg_srv::msg::Motion::SharedPtr msg) const;
    rclcpp::Subscription<motion_msg_srv::msg::Motion>::SharedPtr subscriber_;
};