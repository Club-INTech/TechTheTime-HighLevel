#pragma once

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "../sync/alert_mutex.hpp"
#include <memory>
#include "alert_msg_srv/msg/alert.hpp"

using std::placeholders::_1;

class AlertSubscriber : public rclcpp::Node {

public:
    AlertSubscriber(const std::string& topic, alert_mutex& alert_mut);

private:
    alert_mutex& alert_mut;
    void alert_callback(const alert_msg_srv::msg::Alert::SharedPtr msg) const;
    rclcpp::Subscription<alert_msg_srv::msg::Alert>::SharedPtr subscriber_;
};