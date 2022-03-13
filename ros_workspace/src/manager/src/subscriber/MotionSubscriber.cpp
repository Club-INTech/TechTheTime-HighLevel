#include "MotionSubscriber.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "motion_msg_srv/msg/motion.hpp"

using std::placeholders::_1;

MotionSubscriber::MotionSubscriber() : Node("motion_subscriber") {
    subscriber_ = this->create_subscription<motion_msg_srv::msg::Motion>(
        "motion_topic", 10, std::bind(&MotionSubscriber::motion_callback, this, _1));
}

void MotionSubscriber::motion_callback(const motion_msg_srv::msg::Motion::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "left: %i; right: %i\n", msg->left_ticks, msg->right_ticks);
}