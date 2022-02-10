#include "service/ActionService.hpp"
#include <memory>

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    auto actionService = std::make_shared<ActionService>("action");
    
    rclcpp::spin(actionService);
    rclcpp::shutdown();

    return 0;
}