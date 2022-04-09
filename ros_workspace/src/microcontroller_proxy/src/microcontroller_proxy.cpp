#include "service/ActionService.hpp"
#include "publisher/MotionPublisher.hpp"
#include "alert/AlertSubscriber.hpp"
#include "serial/SerialPort.hpp"
#include <memory>
#include <order/motion.h>
#include <iostream>
#include <thread>
#include <mutex>
#include "bit_decoder/bit_decoder.hpp"
#include "sync/alert_mutex.hpp"
#include "sync/motion_mutex.hpp"

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

MotionMutex::serial_mutex{};
MotionMutex::motion_status_mutex{};
MotionMutex::alert_mutex{};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    std::mutex serial_read_mutex;
    alert_mutex alert_mut{.is_alert = false};
    
    auto serial_port = std::make_shared<scom::SerialPort>("/home/tsimafei/hoho");
    serial_port->open_serial();
    serial_port->get_config();
    serial_port->set_default_config();

    auto motionPublisher = std::make_shared<MotionPublisher>("motion", serial_port, serial_read_mutex, alert_mut);
    auto alertSubscriber = std::make_shared<AlertSubscriber>("alert", alert_mut);
    auto actionService = std::make_shared<ActionService>("action", serial_port, motionPublisher, serial_read_mutex);

    if(argc == 2 && strcmp(argv[1], "monitor") == 0) {
        actionService->microcontroller_gateway->call_remote_function<Motion_Set_Forward_Translation_Setpoint, Shared_Tick>(2000);
        std::this_thread::sleep_for(20ms);
        actionService->microcontroller_gateway->flush();
        while(true) {
            actionService->microcontroller_gateway->call_remote_function<Get_Ticks>();
            std::this_thread::sleep_for(50ms);
            uint64_t value = actionService->microcontroller_gateway->receive_feedback<Get_Ticks>();
            std::this_thread::sleep_for(20ms);
            bit_encoder::values<Get_Ticks, int32_t> decoded_values{};
            decoded_values.decoder.decode(value);
            std::cout << value << " " << decoded_values.decoder.decoded.at(0) << " " << decoded_values.decoder.decoded.at(1) << std::endl;
            std::this_thread::sleep_for(25ms);
        }
    } else if (argc == 1)
    {
        std::thread actionServiceThread([&actionService](){
            rclcpp::spin(actionService);
            actionService->microcontroller_gateway->close_port();
        });

        std::thread motionPublisherThread([&motionPublisher](){
            motionPublisher->broadcast_motion();
        });

        std::thread alertSubscriberThread([&alertSubscriber](){
            rclcpp::spin(alertSubscriber);
        });

        motionPublisherThread.join();
        actionServiceThread.join();
        alertSubscriberThread.join();

    } else {
        std::cout << "Allowed only one argument: [monitor], for ticks monitoring" << std::endl;
    }
    
    rclcpp::shutdown();

    return 0;
}