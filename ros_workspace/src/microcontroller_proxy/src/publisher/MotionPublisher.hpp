#pragma once

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "../serial/SerialPort.hpp"
#include "motion_msg_srv/msg/motion.hpp"
#include "order_codes.hpp"

using namespace std::chrono_literals;

/**
 * @ingroup microcontroller_proxy
 * 
 * MotionPublisher is responsible to request and publish motion feedback provided by microcontroller
 * via UART(see scom::SerialPort).
 * 
 * It also provides a response for ActionService by writing it in MotionPublisher::motion_status
 * 
 * @author sudogauss 
*/ 
class MotionPublisher : public rclcpp::Node {
public:

    /**
     * 
    */ 
    MotionPublisher(std::shared_ptr<scom::SerialPort> gateway);

    /**
     * 
    */ 
    void broadcast_motion(int32_t expected_left_ticks, int32_t expected_right_ticks);

    /**
     * 
    */ 
    int64_t get_motion_status() const;

private:

    /**
     * <a target="_blank" href="">A ROS2 publisher</a> for motion_msg_srv::msg::Motion.
     * To see an example of publisher using see <a target="_blank" href="">there</a> 
    */ 
    rclcpp::Publisher<motion_msg_srv::msg::Motion>::SharedPtr publisher_;

    /**
     * A shared pointer to the scom::SerialPort instance, that has been initialized by ActionService::ActionService
    */ 
    std::shared_ptr<scom::SerialPort> microcontroller_gateway;

    /**
     * Indicates a robot motion status after movement has been finished. It can be of 3 types:
     * COMPLETE, NO_COMPLETE, MOTION_TIMEOUT. See order_codes.hpp
    */ 
    MotionStatusCodes motion_status;

};