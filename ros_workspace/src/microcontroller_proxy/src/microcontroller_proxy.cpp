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
        actionService->microcontroller_gateway->call_remote_function<Get_Left_Ticks>();
        std::this_thread::sleep_for(25ms);
        auto value = actionService->microcontroller_gateway->receive_feedback<Get_Left_Ticks>();
        // Shared_Tick left_ticks = (Shared_Tick) (value >> 32u);
        // Shared_Tick right_ticks = (Shared_Tick) (value & (((Shared_Encoded_Ticks) 1u << 32u) - 1u));
        // std::cout << left_ticks << " " << right_ticks << std::endl;
        std::cout << value << std::endl;
        std::this_thread::sleep_for(25ms);
    }
    actionService->microcontroller_gateway->close_port();
    rclcpp::shutdown();

    return 0;
}