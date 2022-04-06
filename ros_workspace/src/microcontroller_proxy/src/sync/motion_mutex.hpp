#pragma once
#include <mutex>

class MotionMutex {
public:
    
    MotionMutex() {
        serial_mutex = new std::mutex();
        motion_status_mutex = new std::mutex();
    }

    MotionMutex(const MotionMutex& motion_mutex) {
        this->serial_mutex = motion_mutex.serial_mutex;
        this->motion_status_mutex = motion_mutex.motion_status_mutex;
    }   

    ~MotionMutex() {
        delete serial_mutex;
        delete motion_status_mutex;
    }

    template<auto& Ftor, class... Args>
    typename std::result_of<decltype(Ftor)(Args...)>::type protected_call(bool serial_access, 
    bool motion_status_access, Args... args) {
        
        if(serial_access) this->serial_mutex->lock();
        if(motion_status_access) this->motion_status_mutex->lock();

        auto res = Ftor(args...);

        if(serial_access) this->serial_mutex->unlock();
        if(motion_status_access) this->motion_status_mutex->unlock();

        return res;
    }

private:
    std::mutex* serial_mutex;
    std::mutex* motion_status_mutex;
}