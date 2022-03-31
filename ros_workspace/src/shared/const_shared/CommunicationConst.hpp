#pragma once
#include <chrono>

using namespace std::chrono_literals;

/**
 * Period with which <a target="_blank" href="https://github.com/Club-INTech/TechTheTime-LowLevel">LL</a> 
 * is requested to send motion feedback(left and right ticks)
*/ 
constexpr std::chrono::milliseconds MOTION_BROADCAST_PERIOD = 10ms;
constexpr std::chrono::milliseconds READ_FEEDBACK_DELAY = 15ms;

constexpr int64_t TIMEOUT  = 10000;