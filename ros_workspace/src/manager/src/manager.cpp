#include "controller/controllerSetup.hpp"
#include "order/order.hpp"
#include <iostream>
#include <string>
#include <cstddef>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"

#include "subscriber/testSubscriber.hpp"



int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    
    auto test = TestSubscriber();

    rclcpp::shutdown();

    return 0;
}