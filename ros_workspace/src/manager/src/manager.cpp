#include "controller/controllerSetup.hpp"
#include "order/order.hpp"
#include "client/managerClient.hpp"
#include <iostream>
#include <string>
#include <cstddef>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "action_msg_srv/srv/order.hpp"



int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    
    auto commClient = new ClientNode<action_msg_srv::srv::Order, action_msg_srv::srv::Order::Request, std::string, int64_t, int64_t, int64_t, int64_t>("action_client");
    commClient->wait_for_connection();

    rclcpp::shutdown();

    return 0;
}