#pragma once

#include <chrono>
#include <cmath>

/**
 * Acceptable tolerancy in ticks(both for left wheel and right wheel). If 
 * <b>|expected_ticks - ticks| < TICKS_INCERTITUDE</b> then robot is considered to be close enough to its goal.
*/
constexpr int TICKS_INCERTITUDE = 100;

/**
 * Stop condition tolerancy. If <b>|ticks-previous_ticks| < MOTION_CRITERIA</b> then robot considered
 * not moving
*/ 
constexpr int MOTION_CRITERIA = 50;

/**
 * A time within which robot must finish its movement. 
*/ 
constexpr int64_t TIMEOUT  = 10000;

constexpr double WHEEL_DIAMETER_MM = 68.0;
constexpr double TICKS_PER_TURN = 1024;
constexpr double MM_TO_TICKS =  2 * TICKS_PER_TURN / (M_PI * WHEEL_DIAMETER_MM);

using namespace std::chrono_literals;

/**
 * Period with which <a target="_blank" href="https://github.com/Club-INTech/TechTheTime-LowLevel">LL</a> 
 * is requested to send motion feedback(left and right ticks)
*/ 
constexpr std::chrono::milliseconds MOTION_BROADCAST_PERIOD = 20ms;
constexpr std::chrono::milliseconds READ_FEEDBACK_DELAY = 20ms;