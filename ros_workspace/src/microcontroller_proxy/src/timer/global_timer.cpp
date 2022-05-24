#include <timer/global_timer.h>

time_t start_timer = 0;

void init_timer() {
    start_timer = time(NULL);
}

unsigned long elapsed_time() {
    time_t t = time(NULL);
    return (unsigned long) difftime(t, start_timer);
}