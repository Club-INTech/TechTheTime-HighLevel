#include "script.hpp"
#include <action_msg_srv_shared/order_codes.hpp>
#include "../client/ClientT.hpp"
#include <iostream>
#include <string>
#include <cstddef>
#include <stdexcept>
#include <type_traits>
#include "rclcpp/rclcpp.hpp"
#include "../robot_status/RobotStatus.hpp"
#include <cmath>
#include <variant>
#include <functional>
#include <vector>
#include <sstream>
#include <utility>

// By Gaétan Becker (the doc), jtm tim 30%
// Numerotation of XL : FACED VIEW
// ON FLOOR
// 2nd FLOOR : 17 15 13
// 1st FLOOR : 11 9 7
// ON ARM :
// 2nd RAW : 16 14 12
// 1st RAW : 10 8 6
// Numerotation of Pump : FACED VIEW 
// 2nd FLOOR : 5 4 3
// 1st FLOOR : 2 1 0

Script::Script() {
    this->commClient = std::make_shared<ActionClient>();
    this->commClient->set_shared(this->commClient);
    this->commClient->wait_for_connection();

    this->orders.insert({"move", std::function([&](double x, double y){ move(x, y); })});
    this->orders.insert({"moveREL", std::function([&](double d, int recalage){ moveREL(d, recalage); })});
    this->orders.insert({"angleABS", std::function([&](double alpha, int adjustment){ angleABS(alpha, adjustment); })});

    this->orders.insert({"take_statue", std::function([&](){ take_statue(); })});
    this->orders.insert({"drop_replic", std::function([&](){ drop_replic(); })});

    this->orders.insert({"reverse_palet", std::function([&](int id){ reverse_palet(id); })});
    this->orders.insert({"mesure", std::function([&](){ mesure(); })});
    this->orders.insert({"knock_over", std::function([&](){ knock_over(); })});

    this->orders.insert({"down_servos", std::function([&](){ down_servos(); })});
    this->orders.insert({"up_servos", std::function([&](){ up_servos(); })});

    this->orders.insert({"take_palet_vertical", std::function([&](int id){ take_palet_vertical(id); })});
    this->orders.insert({"take_palet_horizontal", std::function([&](int id){ take_palet_horizontal(id); })});
    this->orders.insert({"take_palet_ground", std::function([&](int id){ take_palet_ground(id); })});

    this->orders.insert({"drop_palet_gallery", std::function([&](int id){ drop_palet_gallery(id); })});
    this->orders.insert({"drop_palet_ground", std::function([&](int id){ drop_palet_ground(id); })});

}

void Script::parse_script(const char* script_file) {
    std::ifstream script(script_file);
    std::string unparsed_order;
    while(std::getline(script, unparsed_order)) {
        std::vector<std::string> elems{};
        int sep_index = 0;
        for(int i = 0; i < unparsed_order.size(); i++) {
            if(unparsed_order.at(i) == ' ') {
                elems.push_back(unparsed_order.substr(sep_index, i - sep_index));
                sep_index = i + 1;
            }
        }
        elems.push_back(unparsed_order.substr(sep_index, unparsed_order.size() - sep_index));
        auto order = this->orders.find(elems.at(0));
        if(order == this->orders.end()) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Can't parse: order does not exist");
            return;    
        }

        if(std::holds_alternative<void_profile_T>(order->second)) {
            if(elems.size() != 1) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Can't parse: incorrcet number of arguments for %s", elems.at(0));
                return;                  
            }
            this->pushOrder(std::get<void_profile_T>(order->second));
        } 
        
        else if(std::holds_alternative<move_profile_T>(order->second)) {
            if(elems.size() != 3) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Can't parse: incorrcet number of arguments for %s", elems.at(0));
                return;                  
            }
            this->pushOrder(std::bind(std::get<move_profile_T>(order->second), std::stod(elems.at(1)), std::stod(elems.at(2))));
        } 
        
        else if(std::holds_alternative<actuator_profile_T>(order->second)) {
            if(elems.size() != 3) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Can't parse: incorrcet number of arguments for %s", elems.at(0));
                return;                  
            }
            double command = std::stod(elems.at(1));
            if(elems.at(0) == "angleABS") {
                command = (command / 180) * M_PI;
            }
            this->pushOrder(std::bind(std::get<actuator_profile_T>(order->second), command, std::stoi(elems.at(2))));
        } 
        
        else if(std::holds_alternative<id_profile_T>(order->second)) {
            if(elems.size() != 2) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Can't parse: incorrcet number of arguments for %s", elems.at(0));
                return;                  
            }
            this->pushOrder(std::bind(std::get<id_profile_T>(order->second), std::stoi(elems.at(1))));
        }
    }
}

bool Script::treat_response(MotionStatusCodes status, std::function<void()> OrderToReinsert, bool reinsert = true) {
    if(status == MotionStatusCodes::COMPLETE) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: finished");
        return true;
    } else if(status == MotionStatusCodes::NOT_COMPLETE){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion was not complete");
        if (reinsert){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reinsert the order");
            this->deque_order.push_front(OrderToReinsert);
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
}

void Script::wait_for_jumper() {
    auto res = commClient->send((int64_t) OrderCodes::CHECK_JUMPER, 0, 0, 0);
    res.get();
}

void Script::pushOrder(std::function<void()> orderToPush){
    this->deque_order.push_back(orderToPush);
}

void Script::run(){
    while(!this->deque_order.empty()){
        auto order = this->deque_order.front();
        this->deque_order.pop_front();  
        order();
    }
}

// ================================== Actions orders ============================================================

void Script::take_palet_vertical(int id) { // id = 7 or 9 or 11
    double DBL = -M_PI/12;
    double DBP = M_PI/6;
    int id_pump = 0;
    if(id == 11){
        id_pump = 2;
    }
    else if(id == 9){
        id_pump = 1;
    }
    else{
        id_pump = 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begining of the order - Take palet horizontal with arm %d =====", &id);
    auto res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id-1, DBL);
    res_move_arm_floor.get();
    res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id, DBP);
    res_move_arm_floor.get();
    res_move_arm_floor = commClient->send((int64_t) OrderCodes::ACTIVATE_PUMP, 0, id_pump, 0);
    res_move_arm_floor.get();
    res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id, 0);
    res_move_arm_floor.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Take palet horizontal with arm %d =====", &id);
}

void Script::take_palet_horizontal(int id) { // id = 7 or 9 or 11
    double DBL = -M_PI/2;
    double DBP = 0;
    int id_pump = 0;
    if(id == 11){
        id_pump = 2;
    }
    else if(id == 9){
        id_pump = 1;
    }
    else{
        id_pump = 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begining of the order - Take palet horizontal with arm %d =====", &id);
    auto res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id-1, DBL);
    res_move_arm_floor.get();
    res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id, DBP);
    res_move_arm_floor.get();
    res_move_arm_floor = commClient->send((int64_t) OrderCodes::ACTIVATE_PUMP, 0, id_pump, 0);
    res_move_arm_floor.get();
    res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id, -M_PI/12);
    res_move_arm_floor.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Take palet horizontal with arm %d =====", &id);
}

void Script::take_palet_ground(int id) { // id = 7 or 9 or 11
    double DBL = -M_PI/3;
    double DBP = M_PI/6;
    int id_pump = 0;
    if(id == 11){
        id_pump = 2;
    }
    else if(id == 9){
        id_pump = 1;
    }
    else{
        id_pump = 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begining of the order - Take palet horizontal with arm %d =====", &id);
    auto res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id-1, DBL);
    res_move_arm_floor.get();
    res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id, DBP);
    res_move_arm_floor.get();
    res_move_arm_floor = commClient->send((int64_t) OrderCodes::ACTIVATE_PUMP, 0, id_pump, 0);
    res_move_arm_floor.get();
    res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id, 0);
    res_move_arm_floor.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Take palet horizontal with arm %d =====", &id);
}

void Script::drop_palet_ground(int id) { // id = 7 or 9 or 11
    double DBL = -M_PI/3;
    double DBP = M_PI/6;
    int id_pump = 0;
    if(id == 11){
        id_pump = 2;
    }
    else if(id == 9){
        id_pump = 1;
    }
    else{
        id_pump = 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begining of the order - Take palet horizontal with arm %d =====", &id);
    auto res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id-1, DBL);
    res_move_arm_floor.get();
    res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id, DBP);
    res_move_arm_floor.get();
    res_move_arm_floor = commClient->send((int64_t) OrderCodes::ACTIVATE_PUMP, 0, id_pump, 0);
    res_move_arm_floor.get();
    res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id, -M_PI/12);
    res_move_arm_floor.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Take palet horizontal with arm %d =====", &id);
}

void Script::drop_palet_gallery(int id){ // id = 4 or 5 or 6
    double DBP = -M_PI/6;
    double DBL = -M_PI/6;
    int id_pump = 0;
    if(id == 17){
        id_pump = 2;
    }
    else if(id == 15){
        id_pump = 1;
    }
    else{
        id_pump = 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begining of the order - Drop Palet Gallery with arm %d =====", &id);
    auto res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id, DBP);
    res_move_arm_floor.get();
    res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id-1, DBL);
    res_move_arm_floor.get();
    res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, id, 0);
    res_move_arm_floor.get();
    this->moveREL(130,0);
    res_move_arm_floor = commClient->send((int64_t) OrderCodes::RELEASE_PUMP, 0, id_pump, 0);
    res_move_arm_floor.get();
    this->moveREL(-130,0);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Drop Palet Gallery with arm %d =====", &id);
}

void Script::reverse_palet(int id) { // id = 7 or 9 or 11
    // DBP = Dynamixel Low Near (for the robot)
    // DHL = Dynamixel High Far
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begining of the order - Reverse of all palet arm %d =====", id);
    double DBP = M_PI/12;
    double DBL = 5*M_PI/12;
    double DHL = -5*M_PI/12;
    double DHP = -M_PI/12;

    if(id==7){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "=> Side arm Right <=");

        auto res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 7, DBP);
        res_move_arm_floor.get();
        auto res_move_arm_arm = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 6, DBL);
        res_move_arm_arm.get();
        auto res_move_arm_floor_backup = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 13, DHL);
        res_move_arm_floor_backup.get();
        res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 12, DHP);
        res_move_arm_floor.get();
        auto res_pompe_activ = commClient->send((int64_t) OrderCodes::ACTIVATE_PUMP, 0, 3, 0);
        res_pompe_activ.get();
        res_pompe_activ = commClient->send((int64_t) OrderCodes::RELEASE_PUMP, 0, 0, 0);
        res_pompe_activ.get();
    }
    else if (id == 11){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "=> Side arm Left <=");

        auto res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 11, DBP);
        res_move_arm_floor.get();
        auto res_move_arm_arm = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 10, DBL);
        res_move_arm_arm.get();
        auto res_move_arm_floor_backup = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 17, DHL);
        res_move_arm_floor_backup.get();
        res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 16, DHP);
        res_move_arm_floor.get();
        auto res_pompe_activ = commClient->send((int64_t) OrderCodes::ACTIVATE_PUMP, 0, 5, 0);
        res_pompe_activ.get();
        res_pompe_activ = commClient->send((int64_t) OrderCodes::RELEASE_PUMP, 0, 2, 0);
        res_pompe_activ.get();
    }
    else{
        double DHP_M = -M_PI/6;
        double DHL_M = M_PI/3;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "=> Middle arm <=");

        auto res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 9, DBP);
        res_move_arm_floor.get();
        auto res_move_arm_arm = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 8, DBL);
        res_move_arm_arm.get();
        auto res_move_arm_floor_backup = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 15, DHL);
        res_move_arm_floor_backup.get();
        res_move_arm_floor = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 14, DHP);
        res_move_arm_floor.get();
        auto res_pompe_activ = commClient->send((int64_t) OrderCodes::ACTIVATE_PUMP, 0, 4, 0);
        res_pompe_activ.get();
        res_pompe_activ = commClient->send((int64_t) OrderCodes::RELEASE_PUMP, 0, 1, 0);
        res_pompe_activ.get();
        res_pompe_activ = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 15, DHP_M);
        res_pompe_activ.get();
        res_pompe_activ = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 14, DHL_M);
        res_pompe_activ.get();
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Reverse of all palet arm %d =====", &id);
}

void Script::take_statue(){

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begining of the order - Take Statue =====");

    auto res = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 4, M_PI*(1/2+1/3)); // 0rad of the arm is set at the vertical of the robot
    res.get();
    res = commClient->send((int64_t) OrderCodes::ACTIVATE_PUMP, 0, 13, 0);
    res.get();
    res = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 4, -M_PI*(1/2+1/3)); // 0rad of the arm is set at the vertical of the robot
    res.get();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Take Statue =====");
}

void Script::drop_replic(){

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begining of the order - Drop Replic =====");

    auto res = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 5, -M_PI*(1/2+1/3)); // 0rad of the arm is set at the vertical of the robot
    res.get();
    res = commClient->send((int64_t) OrderCodes::RELEASE_PUMP, 0, 14, 0);
    res.get();
    res = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 5, M_PI*(1+1/3)); // 0rad of the arm is set at the vertical of the robot
    res.get();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Drop Relic =====");
}

void Script::knock_over(){
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begining of the order - Knock over =====");

    auto res = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 15, -M_PI/2); // 0rad of the arm is set at the vertical of the robot
    res.get();
    res = commClient->send((int64_t) OrderCodes::MOVE_ARM, 0, 15, 0);
    res.get();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Knock over =====");

}


void Script::mesure(){

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begining of the order - Read Resistor =====");
    
    auto res = commClient->send((int64_t) OrderCodes::MOVE_SERVO, 0, 2, M_PI/2);
    res.get();

    res = commClient->send((int64_t) OrderCodes::MESURE, 0, 0, 0);
    auto value = res.get()->motion_status;
    
    if(RobotStatus::team == Team::YELLOW &&  800 <= value  && value <= 2000){
        this->knock_over();
    }
    else if(RobotStatus::team == Team::PURPLE && value <= 700){
        this->knock_over();
    }

    res = commClient->send((int64_t) OrderCodes::MOVE_SERVO, 0, 2, 0);
    res.get();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Read Resistor =====");
}

void Script::down_servos(){

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Beging of the order - Down Servos =====");

    auto res = commClient->send((int64_t) OrderCodes::MOVE_SERVO, 0, 1, 3*M_PI/4);
    res.get();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Down Servos =====");

}

void Script::up_servos(){

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Beging of the order - Down Servos =====");

    auto res = commClient->send((int64_t) OrderCodes::MOVE_SERVO, 0, 1, 0);
    res.get();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Down Servos =====");

}

void Script::angleABS(double angle, int readjustment){
    double angle_ = angle;
    angle = fabs(RobotStatus::angle-angle);
    MotionStatusCodes status;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begin of the order - Angle ABS =====");
    if(angle>M_PI){
        if(RobotStatus::angle > angle_) {
            auto res_angle = commClient->send((int64_t) OrderCodes::START_ROTATE_RIGHT, 0, 0, 2*M_PI-angle);    
            status = static_cast<MotionStatusCodes>(res_angle.get()->motion_status);
        } else {
            auto res_angle = commClient->send((int64_t) OrderCodes::START_ROTATE_LEFT, 0, 0, 2*M_PI-angle);
            status = static_cast<MotionStatusCodes>(res_angle.get()->motion_status);
        }
    }
    else{
        if(RobotStatus::angle > angle_) {
            auto res_angle = commClient->send((int64_t) OrderCodes::START_ROTATE_LEFT, 0, 0, angle);    
            status = static_cast<MotionStatusCodes>(res_angle.get()->motion_status);
        } else {
            auto res_angle = commClient->send((int64_t) OrderCodes::START_ROTATE_RIGHT, 0, 0, angle);
            status = static_cast<MotionStatusCodes>(res_angle.get()->motion_status);
        }
    } 

    // Define the order to reinsert
    std::function<void()> orderToReinsert = std::bind(&Script::angleABS, this, angle, 0);
    if(readjustment == 0) {
        this->treat_response(status, orderToReinsert);
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Angle ABS =====");
}

void Script::moveREL(double distance_rel, int recalage){ // recalage = 1 for x and recalage = 2 for y
    double begin_x = RobotStatus::x;
    double begin_y = RobotStatus::y;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begin of the order - Distance ABS =====");
    auto res = commClient->send((int64_t) OrderCodes::MOVE, distance_rel, 0, 0);
    MotionStatusCodes status = static_cast<MotionStatusCodes>(res.get()->motion_status);

    double end_x = RobotStatus::x;
    double end_y = RobotStatus::y;

    // Define the order to reinsert

    double d = sqrt((begin_x - end_x)*(begin_x-end_x) + (begin_y-end_y)*(begin_y-end_y));

    std::function<void()> orderToReinsert = std::bind(&Script::moveREL, this, distance_rel - d, 0);

    if(recalage == 0) {
        this->treat_response(status, orderToReinsert);
    }

    if(RobotStatus::robot == Robot::SLAVE){
        if(recalage ==1){
            if(RobotStatus::x <= 500){
                RobotStatus::x = HALF_LENGTH_SLAVE;
                RobotStatus::angle = M_PI;
            }
            else{
                RobotStatus::x = 3000-HALF_LENGTH_SLAVE;
                RobotStatus::angle = 0;
            }
        }
        else if(recalage == 2){
            if(RobotStatus::y <= 500){
                RobotStatus::y = HALF_LENGTH_SLAVE;
                RobotStatus::angle = 3*M_PI/2;
            }
            else{
                RobotStatus::y = 2000-HALF_LENGTH_SLAVE;
                RobotStatus::angle = M_PI/2;
            }
        }
    }
    else{
        if(recalage == 1){
            if(RobotStatus::x <= 500){
                RobotStatus::x = HALF_LENGTH_MASTER;
                RobotStatus::angle = M_PI;
            }
            else{
                RobotStatus::x = 3000-HALF_LENGTH_MASTER;
                RobotStatus::angle = 0;
            }
        }
        else if(recalage == 2){
            if(RobotStatus::y <= 500){
                RobotStatus::y = HALF_LENGTH_MASTER;
                RobotStatus::angle = 3*M_PI/2;
            }
            else{
                RobotStatus::y = 2000-HALF_LENGTH_MASTER;
                RobotStatus::angle = M_PI/2;
            }
        }
    }


    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Distance ABS =====");
}

void Script::move(double aim_x, double aim_y) {

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Begin of the order - Move =====");
    double curr_x = RobotStatus::x;
    double curr_y = RobotStatus::y;
    double curr_angle = RobotStatus::angle;
    MotionStatusCodes status_angle;
    double distance = sqrt((aim_x-curr_x)*(aim_x-curr_x) + (aim_y-curr_y)*(aim_y-curr_y)); //Physical distance between too position in staight line
    double aim_angle;
    // Voir Feuille Gaétan
    double cosAIM_ANGLE;
    double x = distance*std::cos(curr_angle);
    double y = distance*std::sin(curr_angle);
    double x_prime = aim_x - curr_x;
    double y_prime = aim_y - curr_y;


    cosAIM_ANGLE = (x*x_prime+y*y_prime)/(x*x+y*y);

    aim_angle = std::acos(cosAIM_ANGLE);

    std::cout << aim_angle << std::endl;

    // Pour savoir si on a vraiment le bonne angle on regarde le sinus

    double sinAIM_ANGLE =(y_prime*x - x_prime*y)/(y*y+x*x);

    std::cout << std::sin(aim_angle) << " " << sinAIM_ANGLE << std::endl;

    if(fabs(std::sin(aim_angle)-sinAIM_ANGLE) >= (2 * fabs(std::sin(aim_angle) - 0.1))){
        aim_angle += 2*(M_PI-aim_angle);
    }

    std::cout << aim_angle << std::endl;

    if(aim_angle>M_PI){
        std::cout << 2*M_PI - aim_angle << std::endl;
        auto res_angle = commClient->send((int64_t) OrderCodes::START_ROTATE_LEFT, 0, 0, 2*M_PI-aim_angle);
        status_angle = static_cast<MotionStatusCodes>(res_angle.get()->motion_status);
    }
    else{
        auto res_angle = commClient->send((int64_t) OrderCodes::START_ROTATE_RIGHT, 0, 0, aim_angle);
        status_angle = static_cast<MotionStatusCodes>(res_angle.get()->motion_status);
    }

    // Define the order to reinsert
    std::function<void()> orderToReinsert = std::bind(&Script::move, this, aim_x,aim_y);
    bool execption = this->treat_response(status_angle, orderToReinsert);

    if (execption){
        auto res_move = commClient->send((int64_t) OrderCodes::MOVE, distance, 0, 0);
        MotionStatusCodes status_move = static_cast<MotionStatusCodes>(res_move.get()->motion_status);
        this->treat_response(status_move, orderToReinsert);
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== End of the order - Move =====");
}   