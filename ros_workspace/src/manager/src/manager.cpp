//#include "controller/controllerSetup.hpp"
//#include "order/order.hpp"
//#include "client/ClientT.hpp"
#include "subscriber/testPublisher.hpp"
#include "subscriber/subscriberClass.hpp"
//#include "subscriber/testSubscriber.hpp"
#include <iostream>
#include <string>
#include <cstddef>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
//#include "action_msg_srv/srv/order.hpp"
#include "example_topic/msg/example.hpp"
#include "order_codes.hpp"
#include <array>

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    // auto commClient = std::make_shared<ClientT<action_msg_srv::srv::Order, action_msg_srv::srv::Order::Request, int64_t, int64_t, int64_t, int64_t>>("action");

    // //On dit à ros où envoyer la réponse
    // commClient->set_shared(commClient);
    // commClient->wait_for_connection();

// #ifdef MONTHLERY

//     //On crée une nouvelle classe order qui crée les ordres pour monthléry
//     order::Controlls orders = order::Controlls(commClient);

//     Binder binder = Binder();
//     orders.bindAll(binder);

//     ControllerSetup controller = ControllerSetup(binder);
//     controller.run(true);

// #else

    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::spin(std::make_shared<Subscriber<example_topic::msg::Example>>("aa"));

//#endif

    rclcpp::shutdown();

    return 0;
}