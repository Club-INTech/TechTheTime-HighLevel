#include "service/ActionService.hpp"
#include <memory>
#include <order/motion.h>
#include <iostream>
#include <thread>

/**
 * @ingroup microcontroller_proxy
 * 
 * microcontroller_proxy.cpp is an entry point of the microcontroller_proxy node.
 * 
 * It initializes rclcpp, creates an ActionService and listens for the requests.
 * Take a look to the <a target="_blank" href="https://docs.ros2.org/foxy/api/rclcpp/index.html">rclcpp, the ros2 client library</a>
 * 
 * @author sudogauss
*/

using namespace std::chrono_literals;

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    auto actionService = std::make_shared<ActionService>("action");
    
    // rclcpp::spin(actionService);
    actionService->microcontroller_gateway->call_remote_function<Motion_Set_Forward_Translation_Setpoint, Shared_Tick>(2000);
    while(true) {
        actionService->microcontroller_gateway->call_remote_function<Get_Ticks>();
        auto value = actionService->microcontroller_gateway->receive_feedback<Get_Ticks>();
        // std::cout << (int16_t) (value >> 16) << " " << (int16_t) (value & ((1 << 16) - 1)) << std::endl;
        std::cout << value << std::endl;
        std::this_thread::sleep_for(20ms);
    }
    actionService->microcontroller_gateway->close_port();
    rclcpp::shutdown();

    return 0;
}