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
    
    auto actionClient = ActionClient();
    std::cout << actionClient.send("a", 0, 0, 0, 0) << std::endl;

    rclcpp::shutdown();

    return 0;
}