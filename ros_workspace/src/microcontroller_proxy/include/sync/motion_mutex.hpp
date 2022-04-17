#pragma once

#include <mutex>
#include "alert_mutex.hpp"
#include <functional>
#include <utility>
#include <type_traits>
#include <optional>

namespace motion_mutex {

    extern std::mutex order_mutex;
    extern std::mutex status_mutex;
    extern AlertMutex alert_mutex;

    template<auto Ftor, typename... Args>
    void sync_call(bool order_access, bool status_access,
    bool alert_access, Args&&... args) {

        if(order_access) order_mutex.lock();
        if(status_access) status_mutex.lock();
        if(alert_access) alert_mutex.lock();

        std::invoke(Ftor, std::forward<Args>(args)...);

        if(order_access) order_mutex.unlock();
        if(status_access) status_mutex.unlock();
        if(alert_access) alert_mutex.unlock();
    }
};
