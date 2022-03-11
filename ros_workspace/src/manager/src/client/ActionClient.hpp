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

class ActionClient : public ClientT<action_msg_srv::srv::Order, action_msg_srv::srv::Order::Request, int64_t, int64_t, int64_t, int64_t> {
public:
    ActionClient() : ClientT("action_client") {};

    void treat_response(shared_future_T res) {
        if(res.get()->success) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: finished");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: An error has occurred");
        }
    }
};

#endif