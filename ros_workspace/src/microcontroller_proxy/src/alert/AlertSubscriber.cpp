#include "AlertSubscriber.hpp"

using std::placeholders::_1;

AlertSubscriber::AlertSubscriber(const std::string& topic, alert_mutex& alert_mut) :
    Node("alert_subscriber"), alert_mut(alert_mut) {
        subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            topic, 10, std::bind(&AlertSubscriber::alert_callback, this, _1));
}

void AlertSubscriber::alert_callback(const std_msgs::msg::Bool::SharedPtr msg) const {
    if(msg->data) {
        this->alert_mut.alert();
    } else {
        this->alert_mut.calm();
    }
}