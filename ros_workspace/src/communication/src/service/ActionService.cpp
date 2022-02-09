#include "ActionService.hpp"
#include "rclcpp/rclcpp.hpp"
#include "action_msg_srv/srv/order.hpp"

#include <memory>

ActionService::ActionService(const std::string& service_name) : Node(service_name), service_name(service_name),
    service(this->create_service<action_msg_srv::srv::Order>(service_name, [&](
        const std::shared_ptr<action_msg_srv::srv::Order::Request> req,
        std::shared_ptr<action_msg_srv::srv::Order::Response> res){ this->treat_orders(req, res);})) {}

void ActionService::treat_orders(const std::shared_ptr<action_msg_srv::srv::Order::Request> req,
        std::shared_ptr<action_msg_srv::srv::Order::Response> res) {
        res->success = true;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request: %d\n",
                req->x);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%d]", (bool)res->success);
}