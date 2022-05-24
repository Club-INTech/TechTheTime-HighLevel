#pragma once

#include <const_shared/MotionConst.hpp>
#include <action_msg_srv_shared/order_codes.hpp>
#include <iostream>

enum class Team {
    YELLOW,
    PURPLE,
    NONE
};

enum class Robot {
    MASTER,
    SLAVE,
    NONE
};

class RobotStatus {
public:

    RobotStatus() = delete;

    static double x;
    static double y;
    static double angle_;
    static double angle;
    static Team team;
    static Robot robot;


    static void atomic_move(int64_t left_ticks, int64_t right_ticks) {
        double dbeta = 0; 
        double cos_angle = cos(angle_);
        double sin_angle = sin(angle_);
        if(SIGN(left_ticks) == SIGN(right_ticks)) {
            double c_ticks = SIGN(left_ticks) * MIN(ABS(left_ticks), ABS(right_ticks));
            double dl = c_ticks * TICKS_TO_MM;
            x += (dl * cos_angle);
            y += (dl * sin_angle);
            dbeta = (robot == Robot::MASTER ? TICKS_TO_RADIANS_MASTER : TICKS_TO_RADIANS_SLAVE) * (left_ticks - right_ticks);
        } else if(left_ticks < 0 && right_ticks >= 0) {
            double alpha = MIN(ABS(left_ticks), ABS(right_ticks)) * (robot == Robot::MASTER ? TICKS_TO_RADIANS_HALF_BASE_MASTER : TICKS_TO_RADIANS_HALF_BASE_SLAVE);
            dbeta = -(robot == Robot::MASTER ? TICKS_TO_RADIANS_MASTER : TICKS_TO_RADIANS_SLAVE) * ABS(ABS(left_ticks) - ABS(right_ticks)); 
        } else if(left_ticks >= 0 && right_ticks < 0) {
            double alpha = MIN(ABS(left_ticks), ABS(right_ticks)) * (robot == Robot::MASTER ? TICKS_TO_RADIANS_HALF_BASE_MASTER : TICKS_TO_RADIANS_HALF_BASE_SLAVE);
            angle_ += alpha;
            dbeta = (robot == Robot::MASTER ? TICKS_TO_RADIANS_MASTER : TICKS_TO_RADIANS_SLAVE) * ABS(ABS(left_ticks) - ABS(right_ticks)); 
        }
        angle_ += dbeta;
        double dr = dbeta * (robot == Robot::MASTER ? WHEEL_DISTANCE_MASTER / 2 : WHEEL_DISTANCE_SLAVE / 2);
        x += dr * (1 - dbeta * dbeta / 2);
        y += dr * dbeta;
        double theta = ABS(angle_) - 2 * M_PI * ((int) (ABS(angle_) / (2 * M_PI)));
        angle = (angle_ < 0) ? (2*M_PI - theta) : theta;  
    }
};