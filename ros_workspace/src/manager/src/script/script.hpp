#pragma once

#include "../client/ActionClient.hpp"
#include "action_msg_srv/srv/order.hpp"
#include <action_msg_srv_shared/order_codes.hpp>
#include <functional>
#include <deque>

class Script{
    public:
        Script();
        void move(double, double,int);
        void moveABS(double,double,int);
        void angleABS(double,double,int);
        void take_statue(double,double,int);
        void drop_replic(double,double,int);
        void take_distrib_vertical(double,double,int);
        void reverse_palet(double,double,int);
        void run();
        void pushOrder(std::function<void()>);
        bool treat_response(MotionStatusCodes status, double value, std::function<void()> OrderToReinsert, bool reinsert = true) {
            if(status == MotionStatusCodes::COMPLETE) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: finished with a value of %lf", value);
                return true;
            } else if(status == MotionStatusCodes::NOT_COMPLETE){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion was not complete");
                if (reinsert){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reinsert the order");
                    this->deque_order.push_front(OrderToReinsert); // Already with right parameters - write : OrderToReinsert() to call it
                    return false;
                    }
            } else if(status == MotionStatusCodes::MOTION_TIMEOUT) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion has been timed out");
                if (reinsert){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reinsert the order");
                    this->deque_order.push_front(OrderToReinsert);
                    return false;
                    }
            }
            return true;
        };
    private:
        std::shared_ptr<ActionClient> commClient;
        std::deque<std::function<void()>> deque_order;
};