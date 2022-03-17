#include "MotionSubscriber.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "motion_msg_srv/msg/motion.hpp"
#include "../robot_motion/RobotMotion.hpp"

using std::placeholders::_1;

MotionSubscriber::MotionSubscriber() : Node("motion_subscriber") {
    subscriber_ = this->create_subscription<motion_msg_srv::msg::Motion>(
        "motion_topic", 10, std::bind(&MotionSubscriber::motion_callback, this, _1));
    robot_motion = std::make_unique<RobotMotion>();
}

void MotionSubscriber::motion_callback(const motion_msg_srv::msg::Motion::SharedPtr msg) const {
    this->robot_motion->atomic_move(msg->left_ticks, msg->right_ticks);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Actual position is x: %lf, y: %lf", this->robot_motion->x, this->robot_motion->y);    
}