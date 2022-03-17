#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "motion_msg_srv/msg/motion.hpp"
#include "../robot_motion/RobotMotion.hpp"

using std::placeholders::_1;

class MotionSubscriber : public rclcpp::Node {
public:
    MotionSubscriber();

private:
    void motion_callback(const motion_msg_srv::msg::Motion::SharedPtr msg) const;
    rclcpp::Subscription<motion_msg_srv::msg::Motion>::SharedPtr subscriber_;
    std::unique_ptr<RobotMotion> robot_motion;
};