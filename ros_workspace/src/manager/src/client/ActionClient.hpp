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
#include <action_msg_srv_shared/order_codes.hpp>

class ActionClient : public ClientT<action_msg_srv::srv::Order, action_msg_srv::srv::Order::Request, int64_t, double, int64_t, double> {
public:
    ActionClient() : ClientT("action") {};
};

#endif