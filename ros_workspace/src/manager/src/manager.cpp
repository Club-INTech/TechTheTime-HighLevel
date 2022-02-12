#include "controller/controllerSetup.hpp"
#include "order/order.hpp"
#include "client/ClientT.hpp"
#include <iostream>
#include <string>
#include <cstddef>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "action_msg_srv/srv/order.hpp"
#include "order_codes.hpp"

// #include "subscriber/testSubscriber.hpp"



int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto commClient = std::make_shared<ClientT<action_msg_srv::srv::Order, action_msg_srv::srv::Order::Request,
                int64_t, int64_t, int64_t, int64_t>>("action");
    //On dit à ros où envoyer la réponse
    commClient->set_shared(commClient);
    commClient->wait_for_connection();

    commClient->send(OrderCodes::MOVE, 25, 0, 0);
    

    rclcpp::shutdown();

    return 0;
}