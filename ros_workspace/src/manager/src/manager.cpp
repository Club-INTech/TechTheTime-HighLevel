#include "client/ActionClient.hpp"
#include <iostream>
#include <string>
#include <cstddef>
#include "action_msg_srv/srv/order.hpp"

int main() {
    auto c = new ActionClientNode<action_msg_srv::srv::Order, action_msg_srv::srv::Order::Request, std::string, int64_t, int64_t, int64_t, int64_t>("client_node");
    while(! c->wait_for_connection()) {
        std::cout << "Waiting" << std::endl;
    }
    return 0;
}