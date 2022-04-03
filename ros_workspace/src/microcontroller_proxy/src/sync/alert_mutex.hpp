#pragma once

#include <mutex>

struct alert_mutex {
    bool is_alert;
    std::mutex mut;

    void alert() {
        mut.lock();
        is_alert = true;
        mut.unlock();
    }

    bool alerting() {
        mut.lock();
        bool alert = is_alert;
        mut.unlock();
        return alert;
    }
};