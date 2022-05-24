#ifndef GLOBAL_TIMER_H
#define GLOBAL_TIMER_H

#include <ctime>

extern time_t start_timer;

void init_timer();
unsigned long elapsed_time();

#endif