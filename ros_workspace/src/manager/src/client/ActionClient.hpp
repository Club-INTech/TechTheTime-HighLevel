#ifndef ACTION_CLIENT_HPP
#define ACTION_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "ClientT.hpp"
#include "action_msg_srv/srv/order.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include "struct_wrapper.hpp"
#include <stdexcept>
#include <iostream>
#include "order_codes.hpp"

class ActionClient : public ClientT<action_msg_srv::srv::Order, action_msg_srv::srv::Order::Request, int64_t, int64_t, int64_t, int64_t> {
public:
    ActionClient() : ClientT("action") {};

    void treat_response(shared_future_T res) {
        MotionStatusCodes status = static_cast<MotionStatusCodes>(res.get()->motion_status);
        if(status == MotionStatusCodes::COMPLETE) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: finished");
        } else if(status == MotionStatusCodes::NOT_COMPLETE){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion was not complete");
        } else if(status == MotionStatusCodes::MOTION_TIMEOUT) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion has been timed out");
        }
    }
};

#endif