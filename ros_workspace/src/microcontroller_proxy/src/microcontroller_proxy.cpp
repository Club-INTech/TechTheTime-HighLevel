#include "service/ActionService.hpp"
#include <memory>
#include <order/motion.h>
#include <iostream>
#include <thread>
#include "bit_decoder/bit_decoder.hpp"

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
        std::this_thread::sleep_for(25ms);
        uint64_t value = actionService->microcontroller_gateway->receive_feedback<Get_Ticks>();
        bit_encoder::values<Get_Ticks, int32_t> decoded_values{};
        decoded_values.decoder.decode(value);
        // int32_t left_ticks = (int32_t) (value >> 32);
        // if(left_ticks & ((int32_t) 1 << 20)) {
        //     left_ticks = -(left_ticks & (((int32_t) 1 << 20)-1));
        // }
        // int32_t right_ticks = (int32_t) (value & ((int32_t) ((1 << 32) - 1)));
        // if(right_ticks & ((int32_t) 1 << 20)) {
        //     right_ticks = -(right_ticks & (((int32_t) 1 << 20)-1));
        // }
        // Shared_Tick left_ticks = (Shared_Tick) (value >> 32u);
        // Shared_Tick right_ticks = (Shared_Tick) (value & (((Shared_Encoded_Ticks) 1u << 32u) - 1u));
        // std::cout << left_ticks << " " << right_ticks << std::endl;
        std::cout << value << " " << decoded_values.decoder.decoded.at(0) << " " << decoded_values.decoder.decoded.at(1) << std::endl;
        std::this_thread::sleep_for(25ms);
    }
    actionService->microcontroller_gateway->close_port();
    rclcpp::shutdown();

    return 0;
}