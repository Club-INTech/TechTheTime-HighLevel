#ifndef ACTION_CLIENT_HPP
#define ACTION_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "client/managerClient.hpp"
#include "action_msg_srv/srv/order.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include "struct_wrapper.hpp"
#include <stdexcept>

class ActionClient : public ClientNode<action_msg_srv::srv::Order, action_msg_srv::srv::Order::Request, std::string, int64_t, int64_t, int64_t, int64_t> {
public:
    ActionClient() : ClientNode<>("action_client") {}

    bool send(std::string order, int64_t x, int64_t y, int64_t id, int64_t angle) {
        request->set_values(order, x, y, id, angle);
        auto result = client->async_send_request(request->value);

        //Waiting for result
        if (rclcpp::spin_until_future_complete(this, result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: %d", result.get()->success);
            return result.get()->success;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
            return false;
        }
    }
}