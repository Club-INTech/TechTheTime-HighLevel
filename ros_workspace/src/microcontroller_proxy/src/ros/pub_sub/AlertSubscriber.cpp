#include <ros/pub_sub/AlertSubscriber.hpp>
#include <sync/motion_mutex.hpp>
#include <sync/alert_mutex.hpp>
#include <iostream>

using std::placeholders::_1;

AlertSubscriber::AlertSubscriber(const std::string& topic) : Node("alert_subscriber") {
        subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            topic, 10, std::bind(&AlertSubscriber::alert_callback, this, _1));
}

void AlertSubscriber::alert_callback(const shared_message_T msg) {
    motion_mutex::sync_call<&AlertSubscriber::check_alert>(false, false, true, this);
}

void AlertSubscriber::check_alert() {
    if(msg->data) {
        if(motion_mutex::alert_mutex.alert_status == AlertStatus::CLOSED) {
            motion_mutex::alert_mutex.alert_status = AlertStatus::ALERT;
        } else {
            motion_mutex::alert_mutex.alert_status = AlertStatus::PROCESSING;
        }
    } else {
        motion_mutex::alert_mutex.alert_status = AlertStatus::CLOSED;
    }
}