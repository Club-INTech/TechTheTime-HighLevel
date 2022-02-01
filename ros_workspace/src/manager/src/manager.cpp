#include "controller/controllerSetup.hpp"
#include "order/order.hpp"
#include "client/Client.hpp"
#include <iostream>
#include <string>
#include <cstddef>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "action_msg_srv/srv/order.hpp"
#include "image_msg_srv/srv/imageRequest.hpp"


int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    
    auto commClient = new ClientNode<action_msg_srv::srv::Order, action_msg_srv::srv::Order::Request, std::string, int64_t, int64_t, int64_t, int64_t>("action_client");
    commClient->wait_for_connection();

    auto imgClient = new ClientNode<image_msg_srv::srv::ImageRequest, image_msg_srv::srv::ImageRequest::Request, std::string, int64_t>("image_client");
    imgClient->wait_for_connection();

    rclcpp::shutdown();

    return 0;
}