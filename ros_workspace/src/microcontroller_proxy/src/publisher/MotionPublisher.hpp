#pragma once

#include "rclcpp/rclcpp.hpp"
#include "motion_msg_srv/msg/motion.hpp"
#include "../serial/SerialPort.hpp"
#include <chrono>
#include <mutex>
#include <action_msg_srv_shared/order_codes.hpp>
#include "../sync/alert_mutex.hpp"

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
     * Initializes MotionPublisher.
     * Defines a topic name for MotionPublisher::publisher_ and sets a gateway to microcontroller
     * 
     * @param gateway a shared pointer to the open scom::SerialPort, also known as microcontroller_gateway
    */ 
    MotionPublisher(const std::string& topic, std::shared_ptr<scom::SerialPort> gateway, 
        std::mutex& serial_mut, alert_mutex& alert_mut);

    /**
     * Publishes ticks provided by microcontroller until robot is stopped or finished its goal
     * or timeout for motion has exceeded.
     * 
     * @param expected_left_ticks the left ticks goal
     * @param expected_right_ticks the right ticks goal
    */ 
    void set_motion_goal(int32_t expected_left_ticks, int32_t expected_right_ticks);

    /**
     * @return motion_status for ActionService to build a response
    */ 
    MotionStatusCodes get_motion_status() const;

    void broadcast_motion();



private:

    /**
     * <a target="_blank" href="https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Publisher.html">A ROS2 publisher</a> 
     * for motion_msg_srv::msg::Motion. To see an example of publisher using see 
     * <a target="_blank" href="https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html">there</a> 
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

    std::mutex& serial_read_mutex;

    alert_mutex& alert_mut;

    int32_t expected_left_ticks;
    int32_t expected_right_ticks;
    std::chrono::time_point<std::chrono::system_clock> motion_start;

};