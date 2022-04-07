#pragma once
#include <mutex>

class MotionMutex {
public:
    
    MotionMutex() = delete;
    MotionMutex(const MotionMutex& motion_mutex) = delete;
    MotionMutex(MotionMutex&& motion_mutex) = delete;

    template<auto& Ftor, class... Args>
    static typename std::result_of<decltype(Ftor)(Args...)>::type protected_call(bool serial_access, 
    bool motion_status_access, Args... args) {
        
        if(serial_access) serial_mutex.lock();
        if(motion_status_access) motion_status_mutex.lock();

        auto res = Ftor(args...);

        if(serial_access) serial_mutex.unlock();
        if(motion_status_access) motion_status_mutex.unlock();

        return res;
    }

    static std::mutex serial_mutex;
    static std::mutex motion_status_mutex;
}