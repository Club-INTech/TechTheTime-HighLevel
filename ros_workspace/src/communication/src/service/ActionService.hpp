#ifndef ACTION_SERVER_HPP
#define ACTION_SERVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "action_msg_srv/srv/order.hpp"
#include "order_binder.hpp"

#include <memory>
#include <string>

class ActionService : public rclcpp::Node {

    using request_T = typename action_msg_srv::srv::Order::Request;
    using response_T = typename action_msg_srv::srv::Order::Response;

    using shared_request_T = typename std::shared_ptr<action_msg_srv::srv::Order::Request>;
    using shared_response_T = typename std::shared_ptr<action_msg_srv::srv::Order::Response>;
    
public:
    ActionService(const std::string&);

    void treat_orders(const shared_request_T, shared_response_T);
    

private:
    OrderBinder<shared_request_T, shared_response_T> order_binder;
    std::string service_name;
    rclcpp::Service<action_msg_srv::srv::Order>::SharedPtr service;
};

#endif