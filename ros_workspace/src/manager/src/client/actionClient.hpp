#ifndef ACTION_CLIENT_HPP
#define ACTION_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "clientClass.hpp"
#include "action_msg_srv/srv/order.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include "struct_wrapper.hpp"
#include <stdexcept>
#include <iostream>

class ActionClient : public ClientNode<action_msg_srv::srv::Order, action_msg_srv::srv::Order::Request, std::string, int64_t, int64_t, int64_t, int64_t> {
public:
    ActionClient() : ClientNode("action_client") {};

    void treat() {
        std::cout << "aa"
    }
};

#endif