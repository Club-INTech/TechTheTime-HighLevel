#include <ros/pub_sub/MotionPublisher.hpp>
#include <chrono>
#include <const_shared/CommunicationConst.hpp>
#include <const_shared/MotionConst.hpp>
#include <cmath>
#include <order/motion.h>
#include <com/bit_decoder.hpp>
#include <mutex>
#include <sync/motion_mutex.hpp>


MotionPublisher::MotionPublisher(
    const std::string& topic, 
    std::shared_ptr<scom::SerialPort> gateway) : 
    Node("motion_publisher"),
    microcontroller_gateway(gateway) {

        publisher_ = this->create_publisher<motion_msg_srv::msg::Motion>(topic, 10);
        motion_status = MotionStatusCodes::COMPLETE;
        this->expected_left_ticks = 0;
        this->expected_right_ticks = 0;
        this->left_ticks = 0;
        this->right_ticks = 0;
        this->previous_left_ticks = 0;
        this->previous_right_ticks = 0;
        this->left_ticks_mult = 1;
        this->right_ticks_mult = 1;
        this->motion_start = std::chrono::system_clock::now();

    }

MotionStatusCodes MotionPublisher::get_motion_status() const {
    return this->motion_status;
}

void MotionPublisher::set_motion_goal(int32_t expected_left_ticks, int32_t expected_right_ticks) {
    this->motion_status = MotionStatusCodes::MOVING;
    this->motion_start = std::chrono::system_clock::now();
    this->left_ticks = 0;
    this->right_ticks = 0;
    this->previous_left_ticks = 0;
    this->previous_right_ticks = 0;

    if(expected_left_ticks < 0) {
        this->expected_left_ticks = -expected_left_ticks;
        this->left_ticks_mult = -1;
    } else {
        this->expected_left_ticks = expected_left_ticks;
        this->left_ticks_mult = 1;
    }

    if(expected_right_ticks < 0) {
        this->expected_right_ticks = -expected_right_ticks;
        this->right_ticks_mult = -1;
    } else {
        this->expected_right_ticks = expected_right_ticks;
        this->right_ticks_mult = 1;
    }
}

void MotionPublisher::stop_motion() {
    this->microcontroller_gateway->call_remote_function<Motion_Set_Forward_Translation_Setpoint, Shared_Tick>(0);
    uint8_t buf[5];
    this->microcontroller_gateway->read_word(buf, 5);
    this->expected_left_ticks = 0;
    this->expected_right_ticks = 0;
    this->left_ticks = 0;
    this->right_ticks = 0;
    this->previous_left_ticks = 0;
    this->previous_right_ticks = 0;
    this->left_ticks_mult = 1;
    this->right_ticks_mult = 1;
}

void MotionPublisher::update_status() {
    if(motion_mutex::alert_mutex.alert_status == AlertStatus::ALERT) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[microcontroller_proxy] : Alert");
        this->stop_motion();
        motion_mutex::alert_mutex.alert_status = AlertStatus::PROCESSING;
    }

    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now() - this->motion_start);

    if(
        motion_mutex::alert_mutex.alert_status == AlertStatus::CLOSED && 
        this->motion_status == MOVING && 
        interval.count() >= TIMEOUT) {

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[microcontroller_proxy] : Timed out %d\n", interval.count());
            this->motion_status = MotionStatusCodes::MOTION_TIMEOUT;
            this->stop_motion();
    }

    if(
        motion_mutex::alert_mutex.alert_status == AlertStatus::CLOSED &&  
        abs(this->left_ticks - this->previous_left_ticks) <= MOTION_CRITERIA &&
        abs(this->right_ticks - this->previous_right_ticks) <= MOTION_CRITERIA) {

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[microcontroller_proxy] : Robot stopped");
            
            if(abs(this->left_ticks - this->expected_left_ticks) >= TICKS_INCERTITUDE || 
                (this->right_ticks - this->expected_right_ticks) >= TICKS_INCERTITUDE) {
                this->motion_status = MotionStatusCodes::NOT_COMPLETE;
            } else {
                this->motion_status = MotionStatusCodes::COMPLETE;
            }
        } else this->motion_status = MotionStatusCodes::MOVING;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[microcontroller_proxy] : Status %d", this->motion_status);
}

void MotionPublisher::follow_motion() {

    bool read_error = false;

    auto msg = motion_msg_srv::msg::Motion();

    this->previous_left_ticks = this->left_ticks; 
    this->previous_right_ticks = this->right_ticks;

    this->microcontroller_gateway->call_remote_function<Get_Ticks>();
    auto value = microcontroller_gateway->receive_feedback<Get_Ticks>();
        
    bit_decoder::values<Get_Ticks, int32_t> decoded_values{};
    decoded_values.decoder.decode(value);

    if(decoded_values.decoder.decoded.at(0) < 65536 && 
        decoded_values.decoder.decoded.at(0) > -5000 && 
        decoded_values.decoder.decoded.at(1) < 65536 &&
        decoded_values.decoder.decoded.at(1) > -5000) 
    {

        this->left_ticks = decoded_values.decoder.decoded.at(0);
        this->right_ticks = decoded_values.decoder.decoded.at(1);

    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[microcontroller_proxy] : Read error!\n");
        read_error = true;
    }

    if(!read_error) {
        msg.left_ticks = this->left_ticks_mult * (this->left_ticks - this->previous_left_ticks);
        msg.right_ticks = this->right_ticks_mult * (this->right_ticks - this->previous_right_ticks);
        this->publisher_->publish(msg);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[microcontroller_proxy] : Publishing ticks: %d %d\n", 
        msg.left_ticks, msg.right_ticks);
    }

    motion_mutex::sync_call<&MotionPublisher::update_status>(false, true, true, this);
}


void MotionPublisher::broadcast_motion() {

    while(1) {
        motion_mutex::sync_call<&MotionPublisher::follow_motion>(true, false, false, this);
    }
}
