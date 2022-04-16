#include "script.hpp"
#include <action_msg_srv_shared/order_codes.hpp>
#include "../client/ClientT.hpp"
#include <iostream>
#include <string>
#include <cstddef>
#include <stdexcept>
#include <type_traits>
#include "rclcpp/rclcpp.hpp"
#include "../robot_motion/RobotMotion.hpp"
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

Script::Script() {
    this->commClient = std::make_shared<ActionClient>();
    this->commClient->set_shared(this->commClient);
    this->commClient->wait_for_connection(); 
}

void Script::reverse_palet(double, double, int id) {
    double angle_floor1_floor; // TO DEF
    double angle_floor1_arm; // TO DEF
    double angle_floor1_floor_back; // TO DEF
    double angle_floor1_arm_back; // TO DEF

    double angle_floor2_floor; // TO DEF
    double angle_floor2_arm; // TO DEF
    double angle_floor2_floor_back; // TO DEF
    double angle_floor2_arm_back; // TO DEF

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begining of the order - Reverse the palet of the column %d =====", id);

    auto res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id, angle_floor1_floor);
    auto res_move_arm_arm = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id+6, angle_floor1_arm);
    auto res_pompe_activ = commClient->send((int64_t) OrderCodes::ACTIVATE_PUMP, 0, id, 0);
    auto res_move_arm_floor_backup = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id, angle_floor2_floor);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Take palet with XL %d =====", id);
}


void Script::take_distrib_vertical(double, double, int id) {
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

void Script::take_statue(double, double, int){

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begining of the order - Take Statue =====");

    commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 13, M_PI*(1/2+1/3)); // 0rad of the arm is set at the vertical of the robot
    commClient->send((int64_t) OrderCodes::ACTIVATE_PUMP, 0, 13, 0);
    commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 13, -M_PI*(1/2+1/3)); // 0rad of the arm is set at the vertical of the robot

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Take Statue =====");
}

void Script::drop_replic(double, double, int){

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begining of the order - Drop Replic =====");

    commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 14, -M_PI*(1/2+1/3)); // 0rad of the arm is set at the vertical of the robot
    commClient->send((int64_t) OrderCodes::RELEASE_PUMP, 0, 14, 0);
    commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 13, M_PI*(1+1/3)); // 0rad of the arm is set at the vertical of the robot

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Drop Relic =====");
}

void Script::angleABS(double angle_rel, double, int){
    double angle = -RobotMotion::angle+angle_rel;
    MotionStatusCodes status;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begin of the order - Angle ABS =====");
    if(angle>M_PI){
        auto res_angle = commClient->send((int64_t) OrderCodes::START_ROTATE_LEFT, 0, 0, 2*M_PI-angle);
        status = static_cast<MotionStatusCodes>(res_angle.get()->motion_status);
    }
    else{
        auto res_angle = commClient->send((int64_t) OrderCodes::START_ROTATE_RIGHT, 0, 0, angle);
        status = static_cast<MotionStatusCodes>(res_angle.get()->motion_status);
    } 

    // Define the order to reinsert
    std::function<void()> orderToReinsert = std::bind(&Script::angleABS, this, angle_rel,0.0,0);
    this->treat_response(status,angle,orderToReinsert);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Angle ABS =====");
}

void Script::moveABS(double distance_rel, double, int){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begin of the order - Distance ABS =====");
    auto res = commClient->send((int64_t) OrderCodes::MOVE, distance_rel, 0, 0);
    MotionStatusCodes status = static_cast<MotionStatusCodes>(res.get()->motion_status);

    // Define the order to reinsert
    std::function<void()> orderToReinsert = std::bind(&Script::moveABS, this, distance_rel,0.0,0);
    this->treat_response(status,distance_rel,orderToReinsert);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Distance ABS =====");
}

void Script::run(){
    while(!this->deque_order.empty()){
        auto order = this->deque_order.front();
        this->deque_order.pop_front();  
        order();
    }
}

void Script::move(double aim_x,double aim_y,int) {

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begin of the order - Move =====");
    double curr_x = RobotMotion::x;
    double curr_y = RobotMotion::y;
    double curr_angle = RobotMotion::angle;
    MotionStatusCodes status_angle;
    double distance = sqrt((aim_x-curr_x)*(aim_x-curr_x) + (aim_y-curr_y)*(aim_y-curr_y)); //Physical distance between too position in staight line
    double aim_angle;
    // Voir Feuille Gaétan
    double cosAIM_ANGLE;
    double x = distance*std::cos(curr_angle);
    double y = distance*std::sin(curr_angle);
    double x_prime = aim_x - curr_x;
    double y_prime = aim_y - curr_y;

    std::cout << x_prime << " " << y_prime << std::endl;

    cosAIM_ANGLE = (x*x_prime+y*y_prime)/(x*x+y*y);

    std::cout << cosAIM_ANGLE << std::endl;

    aim_angle = std::acos(cosAIM_ANGLE);

    std::cout << aim_angle << std::endl;

    // Pour savoir si on a vraiment le bonne angle on regarde le sinus

    double sinAIM_ANGLE =(x_prime*y-y_prime*x)/(y*y+x*x);

    if(std::sin(aim_angle)==-sinAIM_ANGLE){
        aim_angle += M_PI;
    }
    if(aim_angle>M_PI){
        auto res_angle = commClient->send((int64_t) OrderCodes::START_ROTATE_LEFT, 0, 0, 2*M_PI-aim_angle);
        status_angle = static_cast<MotionStatusCodes>(res_angle.get()->motion_status);
    }
    else{
        auto res_angle = commClient->send((int64_t) OrderCodes::START_ROTATE_RIGHT, 0, 0, aim_angle);
        status_angle = static_cast<MotionStatusCodes>(res_angle.get()->motion_status);
    }

    // Define the order to reinsert
    std::function<void()> orderToReinsert = std::bind(&Script::move, this, aim_x,aim_y,0);
    bool execption = this->treat_response(status_angle,aim_angle,orderToReinsert);

    if (execption){
        auto res_move = commClient->send((int64_t) OrderCodes::MOVE, distance, 0, 0);
        MotionStatusCodes status_move = static_cast<MotionStatusCodes>(res_move.get()->motion_status);
        this->treat_response(status_move,distance,orderToReinsert);
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Move =====");
}   

void Script::pushOrder(std::function<void()> orderToPush){
    this->deque_order.push_back(orderToPush);
}