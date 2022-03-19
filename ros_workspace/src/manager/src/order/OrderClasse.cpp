#include "OrderClasse.hpp"
#include "controller/controllerSetup.hpp"
#include "order/order.hpp"
#include "client/ClientT.hpp"
#include <iostream>
#include <string>
#include <cstddef>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "order_codes.hpp"
#include "RobotMotion.hpp"
#include <cmath>

// By Gaétan Becker (the doc), jtm tim 30%

Order::Order() {
    auto commClient_MCU = std::make_shared<ActionClient>();
    commClient_MCU->set_shared(commClient_MCU);
    commClient_MCU->wait_for_connection(); 
}

Order::Move(int aim_x,int aim_y) {
    double curr_x = RobotMotion::x;
    double curr_y = RobotMotion::y;
    double curr_angle = RobotMotion::angle;

    double distance = sqrt((aim_x-curr_x)**2 + (aim_y-curr_y)**2); //Physical distance between to position in staight line
    double aim_angle;
    // Voir Feuille Gaétan
    double cosAIM_ANGLE;
    double x = std::cos(curr_angle);
    double y = std::sin(curr_angle);
    double x_prime = aim_x - curr_x;
    double y_prime = aim_y - curr_y;

    cosAIM_ANGLE = (x*x_prime+y*y_prime)/(x**2+y**2);

    aim_angle = std::acos(cosAIM_ANGLE);

    // Pour savoir si on a vraiment le bonne angle on regarde le sinus

    double sinAIM_ANGLE =(x_prime*y-y_prime*x)/(y**2+x**2);

    if(std::asin(aim_angle)==-sinAIM_ANGLE){
        aim_angle += std::M_PI;
    }
    if(aim_angle>std::M_PI){
        auto res_angle = commClient->send((int64_t) OrderCodes::START_ROTATE_LEFT, 0, 0, 2*std::M_PI-aim_angle);
    }
    else{
        auto res_angle = commClient->send((int64_t) OrderCodes::START_ROTATE_RIGHT, 0, 0, aim_angle);
    }
    // Response Treat Rotation
    MotionStatusCodes status_angle = static_cast<MotionStatusCodes>(res_angle.get()->motion_status);
    if(status_angle == MotionStatusCodes::COMPLETE) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: finished with an angle of %lf", &aim_angle);
    } else if(status == MotionStatusCodes::NOT_COMPLETE){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion was not complete");
        return false; // We can avoid this if we look if we tend to get closer to the target with the move  but without the rotate
    } else if(status == MotionStatusCodes::MOTION_TIMEOUT) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion has been timed out");
        return false; // Same thing
    }

    auto res_move = commClient->send((int64_t) OrderCodes::MOVE, distance, 0, 0);

    MotionStatusCodes status_move = static_cast<MotionStatusCodes>(res_move.get()->motion_status);
    if(status_move == MotionStatusCodes::COMPLETE) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: finished with a distance of %lf", &distance);
        return true;
    } else if(status == MotionStatusCodes::NOT_COMPLETE){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion was not complete");
        return false;
    } else if(status == MotionStatusCodes::MOTION_TIMEOUT) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion has been timed out");
        return false;
    }
}