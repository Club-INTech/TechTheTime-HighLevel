#pragma once

#include <cmath>

constexpr double WHEEL_RADIUS_MM = 60.0;
constexpr double TICKS_PER_TURN = 1024;
constexpr double TICKS_TO_MM = WHEEL_RADIUS_MM / TICKS_PER_TURN;
constexpr double HAFL_LENGHT_2A = 75;
constexpr double HAFL_WIDTH_2A = 123;


class RobotMotion {
public:

    RobotMotion() = delete;

    static double x;
    static double y;
    static double angle;

    static void atomic_move(int64_t left_ticks, int64_t right_ticks) {
        x += ((left_ticks * TICKS_TO_MM + right_ticks * TICKS_TO_MM) / 2);
    }
};