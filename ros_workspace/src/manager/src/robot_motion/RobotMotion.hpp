#pragma once

#include <cmath>

#define MIN(a, b) (a) < (b) ? (a) : (b)

constexpr double WHEEL_DIAMETER_MM = 68.0;
constexpr double TICKS_PER_TURN = 1024;
constexpr double TICKS_TO_MM = (M_PI * WHEEL_DIAMETER_MM) / (2 * TICKS_PER_TURN);
constexpr double HAFL_LENGHT_2A = 75;
constexpr double HAFL_WIDTH_2A = 123;
constexpr double RADIUS_BASE_2A = 143.962;
constexpr double HAFL_LENGHT_1A = 83.9925;
constexpr double HAFL_WIDTH_1A = 170;
constexpr double RADIUS_BASE_1A = 184.817;

constexpr double WHEEL_DISTANCE = 278.0;
constexpr double ANGLE_PER_TICK = (M_PI * WHEEL_DIAMETER_MM) / (1024.0 * WHEEL_DISTANCE);

constexpr double WHEEL_DISTANCE = 278.0;
constexpr double ANGLE_PER_TICK = (M_PI * WHEEL_DIAMETER_MM) / (1024.0 * WHEEL_DISTANCE);

class RobotMotion {
public:

    RobotMotion() = delete;

    static double x;
    static double y;
    static double angle;

    static void atomic_move(int64_t left_ticks, int64_t right_ticks) {
        double dl = MIN(left_ticks, right_ticks) * TICKS_TO_MM;
        double delta = right_ticks - left_ticks;
        x += -dl * cos(angle);
        y += -dl * sin(angle);
        angle += ((3 * M_PI_2) - (ANGLE_PER_TICK * delta));
    }
};