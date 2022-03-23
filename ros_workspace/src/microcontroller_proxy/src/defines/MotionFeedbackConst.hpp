#pragma once

#include <chrono>

/**
 * Acceptable tolerancy in ticks(both for left wheel and right wheel). If 
 * <b>|expected_ticks - ticks| < TICKS_INCERTITUDE</b> then robot is considered to be close enough to its goal.
*/
constexpr int TICKS_INCERTITUDE = 20;

/**
 * Stop condition tolerancy. If <b>|ticks-previous_ticks| < MOTION_CRITERIA</b> then robot considered
 * not moving
*/ 
constexpr int MOTION_CRITERIA = 10;

/**
 * A time within which robot must finish its movement. 
*/ 
constexpr int64_t TIMEOUT  = 5000;

using namespace std::chrono_literals;

/**
 * Period with which <a target="_blank" href="https://github.com/Club-INTech/TechTheTime-LowLevel">LL</a> 
 * is requested to send motion feedback(left and right ticks)
*/ 
constexpr std::chrono::milliseconds MOTION_BROADCAST_PERIOD = 50ms;
constexpr std::chrono::milliseconds READ_FEEDBACK_DELAY = 50ms;