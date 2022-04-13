#pragma once

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <sync/motion_mutex.hpp>
#include <memory>
#include <sync/motion_mutex.hpp>
#include <sync/alert_mutex.hpp>
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

class AlertSubscriber : public rclcpp::Node {

public:
    AlertSubscriber(const std::string& topic);

    using shared_message_T = typename std::shared_ptr<std_msgs::msg::Bool>;

private:

    void check_alert();
    void alert_callback(const shared_message_T msg);
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;
};