#pragma once

#include <const_shared/MotionConst.hpp>
#include <action_msg_srv_shared/order_codes.hpp>

class RobotMotion {
public:

    RobotMotion() = delete;

    static double x;
    static double y;
    static double angle;
    static OrderCodes current_order;

    static void atomic_move(int64_t left_ticks, int64_t right_ticks) {
        // TODO: delta ticks influences x, y and angle. 
        if(current_order == OrderCodes::MOVE) {
            double dl = SIGN(left_ticks) * MIN(ABS(left_ticks), ABS(right_ticks)) * TICKS_TO_MM;
            x += dl * cos(angle);
            y += dl * sin(angle);
        } else if(current_order == OrderCodes::START_ROTATE_LEFT) {
            double alpha = MIN(ABS(left_ticks), ABS(right_ticks)) * HALF_TICKS_TO_RADIANS;
            angle -= alpha; 
        } else if(current_order == OrderCodes::START_ROTATE_RIGHT) {
            double alpha = MIN(ABS(left_ticks), ABS(right_ticks)) * HALF_TICKS_TO_RADIANS;
            angle += alpha; 
        }
    }
};