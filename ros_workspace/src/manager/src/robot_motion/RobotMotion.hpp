#pragma once

#include <const_shared/MotionConst.hpp>
#include <action_msg_srv_shared/order_codes.hpp>
#include <iostream>

class RobotMotion {
public:

    RobotMotion() = delete;

    static double x;
    static double y;
    static double angle_;
    static double angle;

    static void atomic_move(int64_t left_ticks, int64_t right_ticks) {
        double dbeta = 0; 
        double cos_angle = cos(angle_);
        double sin_angle = sin(angle_);
        if(SIGN(left_ticks) == SIGN(right_ticks)) {
            double c_ticks = SIGN(left_ticks) * MIN(ABS(left_ticks), ABS(right_ticks));
            double dl = c_ticks * TICKS_TO_MM;
            x += (dl * cos_angle);
            y += (dl * sin_angle);
            dbeta = TICKS_TO_RADIANS * (left_ticks - right_ticks);
        } else if(left_ticks < 0 && right_ticks >= 0) {
            double alpha = MIN(ABS(left_ticks), ABS(right_ticks)) * TICKS_TO_RADIANS_HALF_BASE;
            angle_ -= alpha;
            dbeta = -TICKS_TO_RADIANS * ABS(ABS(left_ticks) - ABS(right_ticks)); 
        } else if(left_ticks >= 0 && right_ticks < 0) {
            double alpha = MIN(ABS(left_ticks), ABS(right_ticks)) * TICKS_TO_RADIANS_HALF_BASE;
            angle_ += alpha;
            dbeta = TICKS_TO_RADIANS * ABS(ABS(left_ticks) - ABS(right_ticks)); 
        }
        angle_ += dbeta;
        double dr = dbeta * (WHEEL_DISTANCE / 2);
        x += dr * (1 - dbeta * dbeta / 2);
        y += dr * dbeta;
        double theta = ABS(angle_) - 2 * M_PI * ((int) (ABS(angle_) / (2 * M_PI)));
        angle = (angle_ < 0) ? (2*M_PI - theta) : theta;  
    }
};