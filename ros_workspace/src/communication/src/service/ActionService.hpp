#ifndef ACTION_SERVER_HPP
#define ACTION_SERVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "action_msg_srv/srv/order.hpp"

#include <memory>
#include <string>

class ActionService : public rclcpp::Node {
    
public:
    ActionService(const std::string&);

    void treat_orders(const std::shared_ptr<action_msg_srv::srv::Order::Request>,
        std::shared_ptr<action_msg_srv::srv::Order::Response>);
    

private:
    std::string service_name;
    rclcpp::Service<action_msg_srv::srv::Order>::SharedPtr service;
};

#endif