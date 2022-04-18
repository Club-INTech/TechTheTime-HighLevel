#include <ros/cli_serv/ActionService.hpp>

#include <action_msg_srv_shared/order_codes.hpp>

#include <chrono>
#include <thread>

#include <order/motion.h>
#include <order/type.h>
#include <sync/alert_mutex.hpp>
#include <sync/motion_mutex.hpp>

#include <const_shared/MotionConst.hpp>
#include <const_shared/CommunicationConst.hpp>

using namespace scom;
using namespace std::chrono_literals;

ActionService::ActionService(
        const std::string& service_name, 
        std::shared_ptr<scom::SerialPort> serial_port,
        std::shared_ptr<MotionPublisher> motion_publisher) : 
        Node(service_name), 
        order_binder(), 
        service_name(service_name) {

                this->service = this->create_service<action_msg_srv::srv::Order>(service_name, 
                        [&](const shared_request_T req, shared_response_T res){ 
                                this->treat_orders(req, res);
                        });

                this->motion_publisher = motion_publisher;
                this->microcontroller_gateway = serial_port;

                order_binder.bind_order(OrderCodes::MOVE, [&](shared_request_T req, shared_response_T res) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving for the distance: %lf\n",req->distance);
                        if(req->distance < 0) {
                                this->microcontroller_gateway->call_remote_function<Motion_Set_Backward_Translation_Setpoint, Shared_Tick>((int32_t) (-MM_TO_TICKS * req->distance));        
                                this->motion_publisher->set_motion_goal((int32_t) (MM_TO_TICKS * req->distance), (int32_t) (MM_TO_TICKS * req->distance));                
                        } else {
                                this->microcontroller_gateway->call_remote_function<Motion_Set_Forward_Translation_Setpoint, Shared_Tick>((int32_t) (MM_TO_TICKS * req->distance));
                                this->motion_publisher->set_motion_goal((int32_t) (MM_TO_TICKS * req->distance), (int32_t) (MM_TO_TICKS * req->distance));
                        }
                });

                order_binder.bind_order(OrderCodes::START_ROTATE_LEFT, [&](shared_request_T req, shared_response_T res) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Turning anticlockwise for angle: %lf\n",req->angle);
                        this->microcontroller_gateway->call_remote_function<Motion_Set_Counterclockwise_Rotation_Setpoint, Shared_Tick>((int32_t) (RADIANS_TO_TICKS_HALF_BASE * req->angle));
                        this->motion_publisher->set_motion_goal((int32_t) (-RADIANS_TO_TICKS_HALF_BASE * req->angle), (int32_t) (RADIANS_TO_TICKS_HALF_BASE * req->angle));
                });

                order_binder.bind_order(OrderCodes::START_ROTATE_RIGHT, [&](shared_request_T req, shared_response_T res) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Turning clockwise for angle: %lf\n",req->angle);
                        this->microcontroller_gateway->call_remote_function<Motion_Set_Clockwise_Rotation_Setpoint, Shared_Tick>((int32_t) (RADIANS_TO_TICKS_HALF_BASE * req->angle));
                        this->motion_publisher->set_motion_goal((int32_t) (RADIANS_TO_TICKS_HALF_BASE * req->angle), (int32_t) (-RADIANS_TO_TICKS_HALF_BASE * req->angle));
                });

        }


void ActionService::execute_order(const shared_request_T req, shared_response_T res) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Order begin");
        this->order_binder.execute_order(req->order_code, req, res);
        uint8_t buf[5];
        this->microcontroller_gateway->read_word(buf, 5);
        std::cout << buf << std::endl;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Order end");
}


void ActionService::treat_orders(const shared_request_T req, shared_response_T res) {
        try {   
                motion_mutex::sync_call<&ActionService::execute_order>(true, false, false, this, req, res);
                res->motion_status = this->spin_while_moving();

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%d]", res->motion_status);
        } catch(const std::runtime_error& e) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", e.what());
        }
}

int64_t ActionService::spin_while_moving() {
        bool status = false;
        while(1) {
                motion_mutex::sync_call<&ActionService::check_motion_status>(false, true, false, this, status);
                if(!status) return (int64_t) this->motion_publisher->get_motion_status();
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting");
                std::this_thread::sleep_for(WAITING_PERIOD);
        }
}

void ActionService::check_motion_status(bool& status) {
        status = (this->motion_publisher->get_motion_status() == MotionStatusCodes::MOVING);
}