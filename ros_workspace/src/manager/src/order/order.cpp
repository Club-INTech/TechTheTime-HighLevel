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
#include <functional>

// By Gaétan Becker (the doc), jtm tim 30%
// Numerotation of XL : FACED VIEW
// ON FLOOR
// 2nd FLOOR : 1 2 3 
// 1st FLOOR : 6 5 4
// ON ARM :
// 2nd RAW : 7 8 9
// 1st RAW : 12 11 10 

Order::Order() {
    commClient = std::make_shared<ActionClient>();
    commClient->set_shared(commClient);
    commClient->wait_for_connection(); 
}

Order::reverse_palet(double useless=0, double useless2=0, int id) {
    double angle_floor1_floor; // TO DEF
    double angle_floor1_arm; // TO DEF
    double angle_floor1_floor_back; // TO DEF
    double angle_floor1_arm_back; // TO DEF

    double angle_floor2_floor; // TO DEF
    double angle_floor2_arm; // TO DEF
    double angle_floor2_floor_back; // TO DEF
    double angle_floor2_arm_back; // TO DEF

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begining of the order - Reverse the palet of the column %d =====", id);

    auto res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id, angle_arm_floor);
    auto res_move_arm_arm = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id+6, angle_arm_arm);
    auto res_pompe_activ = commClient->send((int64_t) OrderCodes::ACTIVATE_PUMP, 0, id, 0);
    auto res_move_arm_floor_backup = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id, angle_arm_floor2);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Take palet with XL %d =====", id);
}


Oder::take_distrib_vertical(double useless=0, double useless=0, int id) {
    double angle_arm_floor; // TO DEF
    double angle_arm_floorback; // TO DEF
    double angle_arm_arm; // TO DEF

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begining of the order - Take palet with XL %d =====", id);

    auto res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id, angle_arm_floor);
    auto res_move_arm_arm = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id+6, angle_arm_arm);
    auto res_pompe_activ = commClient->send((int64_t) OrderCodes::ACTIVATE_PUMP, 0, id, 0);
    auto res_move_arm_floor_backup = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id, angle_arm_floorback);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Take palet with XL %d =====", id);
}

Order::take_statue(double useless=0, double useless2=0, int useless3=0){

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begining of the order - Take Statue =====");

    commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 13, std::M_PI*(1/2+1/3)); // 0rad of the arm is set at the vertical of the robot
    commClient->send((int64_t) OrderCodes::ACTIVATE_PUMP, 0, 13, 0);
    commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 13, -std::M_PI*(1/2+1/3)); // 0rad of the arm is set at the vertical of the robot

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Take Statue =====");
    return true;
}

Order::drop_replic(double useless=0, double useless2=0, int useless3=0){

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begining of the order - Drop Replic =====");

    commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 14, -std::M_PI*(1/2+1/3)); // 0rad of the arm is set at the vertical of the robot
    commClient->send((int64_t) OrderCodes::RELEASE_PUMP, 0, 14, 0);
    commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 13, std::M_PI*(1+1/3)); // 0rad of the arm is set at the vertical of the robot

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Drop Relic =====");
}

Order::angleABS(double angle_rel, double useless=0, double useless2=0){
    double angle = -RobotMotion::angle+angle_rel;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begin of the order - Angle ABS =====");
    if(angle>std::M_PI){
        auto res_angle = commClient->send((int64_t) OrderCodes::START_ROTATE_LEFT, 0, 0, 2*std::M_PI-angle);
    }
    else{
        auto res_angle = commClient->send((int64_t) OrderCodes::START_ROTATE_RIGHT, 0, 0, angle);
    }
    MotionStatusCodes status = static_cast<MotionStatusCodes>(res_angle.get()->motion_status);
    this->treat_response(status,res_angle);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Angle ABS =====");
}

Order::moveABS(double distance_rel, double useless=0, int useless2=0){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begin of the order - Distance ABS =====");
    auto res = commClient->send((int64_t) OrderCodes::MOVE, distance_rel, 0, 0);
    MotionStatusCodes status = static_cast<MotionStatusCodes>(res.get()->motion_status);

    // Define the order to reinsert
    std::function<void(double,double,int)> orderToReinsert = std::bind(&Order::moveABS, this, distance_rel);    // How to choose which argument will be replace in the function is it auto for the elements not pre initialized ?
    this->treat_response(status,distance_rel,orderToReinsert);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Distance ABS =====");
}

Order::move(double aim_x,double aim_y,int useless=0) {

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begin of the order - Move =====");
    double curr_x = RobotMotion::x;
    double curr_y = RobotMotion::y;
    double curr_angle = RobotMotion::angle;

    double distance = sqrt((aim_x-curr_x)**2 + (aim_y-curr_y)**2); //Physical distance between too position in staight line
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
        auto res_angle = commClient->send((int64_t) OrderCodes::START_ROTATE_RIGHT, 0, 0, aim_angle); // Possible de laisser que le IF et de mettre le send à l'extérieur
    }
    MotionStatusCodes status_angle = static_cast<MotionStatusCodes>(res_angle.get()->motion_status);
    this->treat_response(status_angle,aim_angle);

    auto res_move = commClient->send((int64_t) OrderCodes::MOVE, distance, 0, 0);
    MotionStatusCodes status_move = static_cast<MotionStatusCodes>(res_move.get()->motion_status);
    this->treat_response(status_move,distance,Order::move);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Move =====");
}   


