#include "ActionService.hpp"
#include "rclcpp/rclcpp.hpp"
#include "action_msg_srv/srv/order.hpp"

#include <memory>
#include <action_msg_srv_shared/order_binder.hpp>
#include <action_msg_srv_shared/order_codes.hpp>

#include <chrono>
#include <thread>

#include <order/motion.h>
#include <order/type.h>

#include "../serial/SerialPort.hpp"
#include <const_shared/MotionConst.hpp>

using namespace scom;
using namespace std::chrono_literals;

ActionService::ActionService(const std::string& service_name) : Node(service_name), order_binder(), service_name(service_name),
    service(this->create_service<action_msg_srv::srv::Order>(service_name, [&](
        const shared_request_T req, shared_response_T res){ this->treat_orders(req, res);})) {

                microcontroller_gateway = std::make_shared<SerialPort>("/dev/ttyACM0");
                this->microcontroller_gateway->open_serial();
                this->microcontroller_gateway->get_config();
                this->microcontroller_gateway->set_default_config();

                motion_publisher = std::make_shared<MotionPublisher>(this->microcontroller_gateway);

                order_binder.bind_order(OrderCodes::MOVE, [&](shared_request_T req, shared_response_T res) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving for the distance: %lf\n",req->distance);
                        if(req->distance < 0) {
                                this->microcontroller_gateway->call_remote_function<Motion_Set_Backward_Translation_Setpoint, Shared_Tick>((int32_t) (-MM_TO_TICKS * req->distance));        
                        } else {
                                this->microcontroller_gateway->call_remote_function<Motion_Set_Forward_Translation_Setpoint, Shared_Tick>((int32_t) (MM_TO_TICKS * req->distance));
                        }
                        this->motion_publisher->broadcast_motion((int32_t) (MM_TO_TICKS * req->distance), (int32_t) (MM_TO_TICKS * req->distance));
                        res->motion_status = this->motion_publisher->get_motion_status();
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%d]", res->motion_status);                
                });

                order_binder.bind_order(OrderCodes::START_ROTATE_LEFT, [&](shared_request_T req, shared_response_T res) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Turning anticlockwise for angle: %lf\n",req->angle);
                        this->microcontroller_gateway->call_remote_function<Motion_Set_Counterclockwise_Rotation_Setpoint, Shared_Tick>((int32_t) (RADIANS_TO_TICKS * req->angle));
                        this->motion_publisher->broadcast_motion((int32_t) (-RADIANS_TO_TICKS * req->angle), (int32_t) (RADIANS_TO_TICKS * req->angle));
                        res->motion_status = this->motion_publisher->get_motion_status();
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%d]", res->motion_status);                
                });

                order_binder.bind_order(OrderCodes::START_ROTATE_RIGHT, [&](shared_request_T req, shared_response_T res) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Turning clockwise for angle: %lf\n",req->angle);
                        this->microcontroller_gateway->call_remote_function<Motion_Set_Clockwise_Rotation_Setpoint, Shared_Tick>((int32_t) (RADIANS_TO_TICKS * req->angle));
                        this->motion_publisher->broadcast_motion((int32_t) (RADIANS_TO_TICKS * req->angle), (int32_t) (-RADIANS_TO_TICKS * req->angle));
                        res->motion_status = this->motion_publisher->get_motion_status();
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%d]", res->motion_status);                
                });

        }

void ActionService::treat_orders(const shared_request_T req, shared_response_T res) {
        try {
                this->order_binder.execute_order(req->order_code, req, res);
        } catch(const std::runtime_error& e) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", e.what());
        }
}