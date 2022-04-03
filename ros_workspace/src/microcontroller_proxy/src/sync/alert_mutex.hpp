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

    void calm() {
        mut.lock();
        is_alert = false;
        mut.unlock();
    }

    bool alerting() {
        mut.lock();
        bool alert = is_alert;
        mut.unlock();
        return alert;
    }
};