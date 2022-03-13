#include "ActionService.hpp"
#include "rclcpp/rclcpp.hpp"
#include "action_msg_srv/srv/order.hpp"

#include <memory>
#include "order_binder.hpp"
#include "order_codes.hpp"

#include <chrono>
#include <thread>

#include <order/motion.h>
#include <order/type.h>

#include "../serial/SerialPort.hpp"

using namespace scom;

ActionService::ActionService(const std::string& service_name) : Node(service_name), order_binder(), service_name(service_name),
    service(this->create_service<action_msg_srv::srv::Order>(service_name, [&](
        const shared_request_T req, shared_response_T res){ this->treat_orders(req, res);})) {

                this->microcontroller_gateway = std::make_unique<SerialPort>("/dummy");
                this->microcontroller_gateway->open_serial();
                this->microcontroller_gateway->get_config();
                this->microcontroller_gateway->set_default_config();

                order_binder.bind_order(OrderCodes::MOVE, [&](shared_request_T req, shared_response_T res) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving for the distance: %d\n",req->distance);
                        this->microcontroller_gateway->call_remote_function<Motion_Set_Forward_Translation_Setpoint, Shared_Tick>(3*req->distance);
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                        res->success = true;
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%d]", (bool)res->success);                
                });

        }

void ActionService::treat_orders(const shared_request_t req, shared_response_T res) {
        this->order_binder.execute_order(req->order_code, req, res);
}