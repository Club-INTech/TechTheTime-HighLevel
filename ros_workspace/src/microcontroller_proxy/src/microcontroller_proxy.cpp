#include <ros/cli_serv/ActionService.hpp>
#include <ros/pub_sub/MotionPublisher.hpp>
#include <ros/pub_sub/AlertSubscriber.hpp>
#include <com/SerialPort.hpp>
#include <com/bit_decoder.hpp>
#include <memory>
#include <order/motion.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <sync/alert_mutex.hpp>
#include <sync/motion_mutex.hpp>
#include "yaml-cpp/yaml.h"
#include <csignal>
#include <fstream>
#include <const_shared/CommunicationConst.hpp>

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

std::mutex motion_mutex::order_mutex{};
std::mutex motion_mutex::status_mutex{};
AlertMutex motion_mutex::alert_mutex{.alert_status = AlertStatus::CLOSED};

void terminate(int code) {
    rclcpp::shutdown();
    exit(code);
}

template<typename T>
T process_element(YAML::Node* config, const char* elem) {
    if(!(*config)[elem]) {
        std::cout << "Parameter " << elem << " is required" << std::endl;
        terminate(1);
    } else {
        return (*config)[elem].as<T>();
    } 
}

int main(int argc, char** argv) {

    std::signal(SIGINT, terminate);

    rclcpp::init(argc, argv);

    if(argc < 2) {
        std::cout << "Please provide a full path to your config file. Example: $PWD/config.yaml" << std::endl;
        terminate(1);
    }

    std::string filename(argv[1]);
    YAML::Node config = YAML::LoadFile(filename);
    
    auto serial_port = std::make_shared<scom::SerialPort>(process_element<std::string>(&config, "serial_port").c_str());
    serial_port->open_serial();
    serial_port->set_default_config();

    auto motionPublisher = std::make_shared<MotionPublisher>(
        process_element<std::string>(&config, "motion_topic"), 
        serial_port
    );

    auto alertSubscriber = std::make_shared<AlertSubscriber>(
        process_element<std::string>(&config, "alert_topic")
    );
    auto actionService = std::make_shared<ActionService>(
        process_element<std::string>(&config, "action_topic"),
        process_element<std::string>(&config, "robot"), 
        serial_port, 
        motionPublisher
    );

    auto mode = process_element<std::string>(&config, "mode");

    if(mode == "monitor") {

        actionService->microcontroller_gateway->call_remote_function<Motion_Set_Forward_Translation_Setpoint, Shared_Tick>(2000);
        std::this_thread::sleep_for(10ms);
        actionService->microcontroller_gateway->flush();
        while(true) {
            actionService->microcontroller_gateway->call_remote_function<Get_Ticks>();
            std::this_thread::sleep_for(20ms);
            uint64_t value = actionService->microcontroller_gateway->receive_feedback<Get_Ticks>();
            bit_decoder::values<Get_Ticks, int32_t> decoded_values{};
            decoded_values.decoder.decode(value);
            std::cout << value << " " << decoded_values.decoder.decoded.at(0) << " " << decoded_values.decoder.decoded.at(1) << std::endl;
            std::this_thread::sleep_for(10ms);
        }

    } else if(mode == "match") {

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
        std::cout << "The following arguments are required: [monitor, match]" << std::endl;
    }
    
    rclcpp::shutdown();

    return 0;
}