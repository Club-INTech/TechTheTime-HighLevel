#include "service/ActionService.hpp"
#include <memory>

/**
 * microcontroller_proxy.cpp is an entry point of the microcontroller_proxy node.
 * 
 * It initializes rclcpp, creates an actionService {@link service/ActionService.hpp} and listens for the requests.
 * @author sudogauss
*/

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    auto actionService = std::make_shared<ActionService>("action");
    
    rclcpp::spin(actionService);
    rclcpp::shutdown();

    return 0;
}