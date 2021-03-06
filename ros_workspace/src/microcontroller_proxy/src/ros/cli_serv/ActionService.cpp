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

#include <timer/global_timer.h>

using namespace scom;
using namespace std::chrono_literals;

ActionService::ActionService(
        const std::string& service_name, 
        const std::string& robot,
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

                this->start_reception = false;

                double RADIANS_TO_TICKS_HALF_BASE = (robot == "master" ? RADIANS_TO_TICKS_HALF_BASE_MASTER : RADIANS_TO_TICKS_HALF_BASE_SLAVE);

                order_binder.bind_order(OrderCodes::MOVE, [&](shared_request_T req, shared_response_T res) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[microcontroller_proxy] : Moving for the distance: %lf\n",req->distance);
                        if(req->distance < 0) {
                                this->microcontroller_gateway->call_remote_function<Motion_Set_Backward_Translation_Setpoint, Shared_Tick>((int32_t) (-MM_TO_TICKS * req->distance));        
                                this->motion_publisher->set_motion_goal((int32_t) (MM_TO_TICKS * req->distance), (int32_t) (MM_TO_TICKS * req->distance));                
                        } else {
                                this->microcontroller_gateway->call_remote_function<Motion_Set_Forward_Translation_Setpoint, Shared_Tick>((int32_t) (MM_TO_TICKS * req->distance));
                                this->motion_publisher->set_motion_goal((int32_t) (MM_TO_TICKS * req->distance), (int32_t) (MM_TO_TICKS * req->distance));
                        }
                });

                order_binder.bind_order(OrderCodes::START_ROTATE_LEFT, [&](shared_request_T req, shared_response_T res) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[microcontroller_proxy] : Turning anticlockwise for angle: %lf\n",req->angle);
                        this->microcontroller_gateway->call_remote_function<Motion_Set_Counterclockwise_Rotation_Setpoint, Shared_Tick>((int32_t) (RADIANS_TO_TICKS_HALF_BASE * req->angle));
                        this->motion_publisher->set_motion_goal((int32_t) (-RADIANS_TO_TICKS_HALF_BASE * req->angle), (int32_t) (RADIANS_TO_TICKS_HALF_BASE * req->angle));
                });

                order_binder.bind_order(OrderCodes::START_ROTATE_RIGHT, [&](shared_request_T req, shared_response_T res) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[microcontroller_proxy] : Turning clockwise for angle: %lf\n",req->angle);
                        this->microcontroller_gateway->call_remote_function<Motion_Set_Clockwise_Rotation_Setpoint, Shared_Tick>((int32_t) (RADIANS_TO_TICKS_HALF_BASE * req->angle));
                        this->motion_publisher->set_motion_goal((int32_t) (RADIANS_TO_TICKS_HALF_BASE * req->angle), (int32_t) (-RADIANS_TO_TICKS_HALF_BASE * req->angle));
                });

                order_binder.bind_order(OrderCodes::CHECK_JUMPER, [&](shared_request_T req, shared_response_T res) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[microcontroller_proxy] : Checking jumper\n");
                        while(1) {
                                this->microcontroller_gateway->call_remote_function<isJumperOn>();
                                auto value = this->microcontroller_gateway->receive_feedback<isJumperOn>();
                                if(value) break;
                                // std::this_thread::sleep_for(JUMPER_WAITING);
                                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[microcontroller_proxy] : Waiting jumper %d\n", value);
                        }
                        init_timer();
                        this->start_reception = true;
                        res->motion_status = (int64_t) MotionStatusCodes::COMPLETE; 
                });

                order_binder.bind_order(OrderCodes::MOVE_ARM, [&](shared_request_T req, shared_response_T res) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[microcontroller_proxy] : Moving arm\n");
                        this->microcontroller_gateway->call_remote_function<DXL_Position_Angle>(static_cast<uint8_t>(req->id), static_cast<int32_t>(180 * req->angle / M_PI));
                        std::this_thread::sleep_for(ARM_WAITING_PERIOD);
                        res->motion_status = (int64_t) MotionStatusCodes::COMPLETE; 
                });

                order_binder.bind_order(OrderCodes::ACTIVATE_PUMP, [&](shared_request_T req, shared_response_T res) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[microcontroller_proxy] : Activating pump\n");
                        this->microcontroller_gateway->call_remote_function<Misc_Set_Valve>(static_cast<uint8_t>(req->id), 0);
                        uint8_t buf[5];
                        this->microcontroller_gateway->read_word(buf, 5);
                        std::cout << buf << std::endl;
                        this->microcontroller_gateway->call_remote_function<Misc_Set_Pump>(static_cast<uint8_t>(req->id), 1);
                        std::this_thread::sleep_for(PUMP_WAITING_PERIOD);
                        res->motion_status = (int64_t) MotionStatusCodes::COMPLETE; 
                });

                order_binder.bind_order(OrderCodes::RELEASE_PUMP, [&](shared_request_T req, shared_response_T res) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[microcontroller_proxy] : Releasing pump\n");
                        this->microcontroller_gateway->call_remote_function<Misc_Set_Pump>(static_cast<uint8_t>(req->id), 0);
                        uint8_t buf[5];
                        this->microcontroller_gateway->read_word(buf, 5);
                        std::cout << buf << std::endl;
                        this->microcontroller_gateway->call_remote_function<Misc_Set_Valve>(static_cast<uint8_t>(req->id), 1);
                        std::this_thread::sleep_for(PUMP_WAITING_PERIOD);
                        res->motion_status = (int64_t) MotionStatusCodes::COMPLETE; 
                });

                order_binder.bind_order(OrderCodes::MOVE_SERVO, [&](shared_request_T req, shared_response_T res) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[microcontroller_proxy] : Moving servo\n");
                        this->microcontroller_gateway->call_remote_function<Misc_Set_Servo>(static_cast<uint8_t>(req->id), static_cast<uint16_t>(65535 * req->angle / M_PI));
                        std::this_thread::sleep_for(ARM_WAITING_PERIOD);
                        res->motion_status = (int64_t) MotionStatusCodes::COMPLETE; 
                });

                order_binder.bind_order(OrderCodes::MESURE, [&](shared_request_T req, shared_response_T res) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[microcontroller_proxy] : Resistance measuring\n");
                        this->microcontroller_gateway->call_remote_function<giveRes>();
                        std::this_thread::sleep_for(MEASURE_WAITING_PERIOD);
                        auto measure = this->microcontroller_gateway->receive_feedback<giveRes>();
                        res->motion_status = (int64_t) measure; 
                });

}


void ActionService::execute_order(const shared_request_T req, shared_response_T res) {
        if(req->order_code <= 7) {
                this->order_binder.execute_order(req->order_code, req, res);
                uint8_t buf[5];
                this->microcontroller_gateway->read_word(buf, 5);
                std::cout << buf << std::endl;
        } else {
                this->order_binder.execute_order(req->order_code, req, res);
        }
}


void ActionService::treat_orders(const shared_request_T req, shared_response_T res) {

        if(elapsed_time() >= MATCH_TIME && this->start_reception) {
                while(1) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[microcontroller_proxy] : Finished match");
                }
        }

        try {   
                if(req->order_code <= 3) {
                        motion_mutex::sync_call<&ActionService::execute_order>(true, false, false, this, req, res);
                        res->motion_status = this->spin_while_moving();
                } else  {
                        motion_mutex::sync_call<&ActionService::execute_order>(true, false, false, this, req, res);
                }

        } catch(const std::runtime_error& e) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[microcontroller_proxy] : %s", e.what());
        }
}

int64_t ActionService::spin_while_moving() {
        bool status = false;
        while(1) {
                motion_mutex::sync_call<&ActionService::check_motion_status>(false, true, false, this, status);
                if(!status) return (int64_t) this->motion_publisher->get_motion_status();
                std::this_thread::sleep_for(WAITING_PERIOD);
        }
}

void ActionService::check_motion_status(bool& status) {
        status = (this->motion_publisher->get_motion_status() == MotionStatusCodes::MOVING);
}