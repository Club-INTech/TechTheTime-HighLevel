#pragma once

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "../sync/alert_mutex.hpp"
#include <memory>
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

class AlertSubscriber : public rclcpp::Node {

public:
    AlertSubscriber(const std::string& topic, alert_mutex& alert_mut);

private:
    alert_mutex& alert_mut;
    void alert_callback(const std_msgs::msg::Bool::SharedPtr msg) const;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;
};