#pragma once

#include <chrono>

#define TICKS_INCERTITUDE 20
#define MOTION_CRITERIA 10
#define TIMEOUT 5000

using namespace std::chrono_literals;

constexpr std::chrono::milliseconds MOTION_BROADCAST_PERIOD = 100ms;