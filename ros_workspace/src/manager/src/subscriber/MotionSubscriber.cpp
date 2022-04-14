#include "MotionSubscriber.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "motion_msg_srv/msg/motion.hpp"
#include "../robot_motion/RobotMotion.hpp"

using std::placeholders::_1;

MotionSubscriber::MotionSubscriber() : Node("motion_subscriber") {
    subscriber_ = this->create_subscription<motion_msg_srv::msg::Motion>(
        "motion", 10, std::bind(&MotionSubscriber::motion_callback, this, _1));
}

void MotionSubscriber::motion_callback(const motion_msg_srv::msg::Motion::SharedPtr msg) const {
    RobotMotion::atomic_move(msg->left_ticks, msg->right_ticks);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Actual position is x: %lf, y: %lf, angle: %lf", 
        RobotMotion::x, RobotMotion::y, RobotMotion::angle);    
}