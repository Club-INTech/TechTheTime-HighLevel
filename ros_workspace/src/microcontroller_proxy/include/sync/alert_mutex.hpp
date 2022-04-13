#pragma once

#include <mutex>

enum AlertStatus {
    ALERT,
    PROCESSING,
    CLOSED
};

struct AlertMutex {
    AlertStatus alert_status;
    std::mutex mut;

    void lock() {
        mut.lock();
    }

    void unlock() {
        mut.unlock();
    }
};