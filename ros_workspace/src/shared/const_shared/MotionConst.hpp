#pragma once

#include <cmath>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define ABS(a) ((a) < 0 ? (-(a)) : (a))
#define SIGN(a) ((a) < 0 ? (-1) : (1))

/**
 * Acceptable tolerancy in ticks(both for left wheel and right wheel). If 
 * <b>|expected_ticks - ticks| < TICKS_INCERTITUDE</b> then robot is considered to be close enough to its goal.
*/
constexpr int TICKS_INCERTITUDE = 100;

/**
 * Stop condition tolerancy. If <b>|ticks-previous_ticks| < MOTION_CRITERIA</b> then robot considered
 * not moving
*/ 
constexpr int MOTION_CRITERIA = 0;

/**
 * A time within which robot must finish its movement. 
*/ 

constexpr double WHEEL_DIAMETER_MM = 68.0;
constexpr double TICKS_PER_TURN = 1024;
constexpr double HALF_LENGTH_SLAVE = 75;
constexpr double HALF_WIDTH_SLAVE = 123;
constexpr double RADIUS_BASE_SLAVE = 143.962;
constexpr double HALF_LENGTH_MASTER = 83.9925;
constexpr double HALF_WIDTH_MASTER = 170;
constexpr double RADIUS_BASE_MASTER = 184.817;

constexpr double WHEEL_DISTANCE_SLAVE = 278.0;
constexpr double WHEEL_DISTANCE_MASTER = 331.0;
constexpr double HALF_WHEEL_DISTANCE_SLAVE = WHEEL_DISTANCE_SLAVE / 2;
constexpr double HALF_WHEEL_DISTANCE_MASTER = WHEEL_DISTANCE_MASTER / 2;

constexpr double TICKS_TO_MM = (M_PI * WHEEL_DIAMETER_MM) / (2 * TICKS_PER_TURN);
constexpr double MM_TO_TICKS =  1 / TICKS_TO_MM;

constexpr double TICKS_TO_RADIANS_SLAVE = TICKS_TO_MM / WHEEL_DISTANCE_SLAVE;
constexpr double RADIANS_TO_TICKS_SLAVE = 1 / TICKS_TO_RADIANS_SLAVE;
constexpr double TICKS_TO_RADIANS_MASTER = TICKS_TO_MM / WHEEL_DISTANCE_MASTER;
constexpr double RADIANS_TO_TICKS_MASTER = 1 / TICKS_TO_RADIANS_MASTER;

constexpr double TICKS_TO_RADIANS_HALF_BASE_SLAVE = 2 * TICKS_TO_RADIANS_SLAVE;
constexpr double RADIANS_TO_TICKS_HALF_BASE_SLAVE = 1 / TICKS_TO_RADIANS_HALF_BASE_SLAVE;
constexpr double TICKS_TO_RADIANS_HALF_BASE_MASTER = 2 * TICKS_TO_RADIANS_MASTER;
constexpr double RADIANS_TO_TICKS_HALF_BASE_MASTER = 1 / TICKS_TO_RADIANS_HALF_BASE_MASTER;

constexpr double START_X_2A_YELLOW = 257.0;
constexpr double START_Y_2A_YELLOW = 915.0;
constexpr double START_ANGLE_2A_YELLOW = M_PI/2;

constexpr double START_X_2A_PURPLE = 2743;
constexpr double START_Y_2A_PURPLE = 915.0;
constexpr double START_ANGLE_2A_PURPLE = M_PI/2;

constexpr double START_X_1A_YELLOW = 119.0;
constexpr double START_Y_1A_YELLOW = 577.0;
constexpr double START_ANGLE_1A_YELLOW = 0;

constexpr double START_X_1A_PURPLE = 2820.0;
constexpr double START_Y_1A_PURPLE = 750;
constexpr double START_ANGLE_1A_PURPLE= M_PI;

constexpr double MOVE_PRECISION = 900.0;
constexpr double ROTATION_PRECISION = 5.0 * M_PI / 180.0;