#pragma once

#include "../client/ActionClient.hpp"
#include "action_msg_srv/srv/order.hpp"
#include "order_codes.hpp"
#include <functional>

class Order{
    auto deck_order;
    public:
        Order();
        void move(double, double,int);
        void moveABS(double,double,int);
        void angleABS(double,double,int);
        void take_statue(double,double,int);
        void drop_replic(double,double,int);
        void take_palet_distrib(double,double,int);
        void treat_response(MotionStatusCodes status, double value=12321, std::function<void(double,double,int)> OrderToReinsert, bool reinsert = true) {
            if(status == MotionStatusCodes::COMPLETE) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: finished with a value of %lf", value);
            } else if(status == MotionStatusCodes::NOT_COMPLETE){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion was not complete");
                if (reinsert){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reinsert the order");
                    this.deck_order.push_front(OrderToReinsert); // Already with right parameters - write : OrderToReinsert() to call it
            }
            } else if(status == MotionStatusCodes::MOTION_TIMEOUT) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion has been timed out");
                if (reinsert){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reinsert the order");
                    this.deck_order.push_front(OrderToReinsert);
        }
    }
}

    private:
        auto commClient;
};
// std::function<void(const Foo&, int)> f_add_display = &Foo::print_add;