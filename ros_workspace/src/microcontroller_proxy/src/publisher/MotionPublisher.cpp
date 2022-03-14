#include "MotionPublisher.hpp"
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "../serial/SerialPort.hpp"
#include "motion_msg_srv/msg/motion.hpp"
#include "../defines/TimeConst.hpp"
#include <cmath>

MotionPublisher::MotionPublisher(std::shared_ptr<scom::SerialPort> gateway) : 
    microcontroller_gateway(gateway), Node("motion_publisher") {
        publisher_ = this->create_publisher<motion_msg_srv::msg::Motion>("motion_topic", 10);
    }


void MotionPublisher::broadcast_motion(int32_t expected_left_ticks, int32_t expected_right_ticks) {
    int32_t left_ticks = 0, right_ticks = 0;
    int32_t previous_left_ticks = MOTION_CRITERIA, previous_right_ticks = MOTION_CRITERIA;
    auto start = std::chrono::system_clock::now();
    while(
        abs(left_ticks - expected_left_ticks) >= TICKS_INCERTITUDE &&
        abs(right_ticks - expected_right_ticks) >= TICKS_INCERTITUDE &&
        abs(left_ticks - previous_left_ticks) >= MOTION_CRITERIA &&
        abs(right_ticks - previous_right_ticks) >= MOTION_CRITERIA
        ) {
            previous_left_ticks = left_ticks;
            previous_right_ticks = right_ticks; 
            left_ticks = (left_ticks + 200 >= expected_left_ticks ? expected_left_ticks : (left_ticks + 200));
            left_ticks = (right_ticks + 200 >= expected_right_ticks ? expected_right_ticks : (right_ticks + 200));
            auto now = std::chrono::system_clock::now();
            auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
            if(interval.count() >= TIMEOUT) {
                break;
            }
            auto msg = motion_msg_srv::msg::Motion();
            msg.left_ticks = 200;
            msg.right_ticks = 200;
            this->publisher_->publish(msg);
            std::this_thread::sleep_for(MOTION_BROADCAST_PERIOD);
        }
}
