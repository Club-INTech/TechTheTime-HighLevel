#include "AlertSubscriber.hpp"

using std::placeholders::_1;

AlertSubscriber::AlertSubscriber(const std::string& topic, alert_mutex& alert_mut) :
    Node("alert_subscriber"), alert_mut(alert_mut) {
        subscriber_ = this->create_subscription<alert_msg_srv::msg::Alert>(
            topic, 10, std::bind(&AlertSubscriber::alert_callback, this, _1));
}

void AlertSubscriber::alert_callback(const alert_msg_srv::msg::Alert::SharedPtr msg) const {
    if(msg->alert) {
        this->alert_mut.alert();
    } else {
        this->alert_mut.calm();
    }
}