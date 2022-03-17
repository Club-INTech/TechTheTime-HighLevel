#pragma once

#include <chrono>

/**
 *  
*/
constexpr int TICKS_INCERTITUDE = 20;
constexpr int MOTION_CRITERIA = 10;
constexpr int64_t TIMEOUT  = 4000;

using namespace std::chrono_literals;

constexpr std::chrono::milliseconds MOTION_BROADCAST_PERIOD = 20ms;