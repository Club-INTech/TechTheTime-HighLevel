#pragma once
#include <chrono>

using namespace std::chrono_literals;

/**
 * Period with which <a target="_blank" href="https://github.com/Club-INTech/TechTheTime-LowLevel">LL</a> 
 * is requested to send motion feedback(left and right ticks)
*/ 
constexpr std::chrono::milliseconds SERIAL_COM_DELAY = 50ms;
constexpr std::chrono::milliseconds MOTION_BROADCAST_PERIOD = 0ms;
constexpr std::chrono::milliseconds READ_FEEDBACK_DELAY = 0ms;

constexpr std::chrono::milliseconds WAITING_PERIOD = 100ms;

constexpr std::chrono::milliseconds JUMPER_WAITING = 20ms;

constexpr std::chrono::milliseconds ARM_WAITING_PERIOD = 1000ms;
constexpr std::chrono::milliseconds PUMP_WAITING_PERIOD = 1000ms;

constexpr std::chrono::milliseconds MEASURE_WAITING_PERIOD = 2000ms;

constexpr int64_t TIMEOUT  = 10000;

constexpr unsigned long MATCH_TIME = 95;