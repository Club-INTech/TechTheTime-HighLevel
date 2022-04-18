#pragma once

#include <mutex>

/**
 * @ingroup microcontroller_proxy
 * AlertStatus indicates a current status of alert, when the other robot was detected
 */
enum AlertStatus {
    /**
     * Signifies that alert has been envoked and not processed.
     */
    ALERT,
    /**
     * Signifies that alert has been already envoked and is in process.
     */
    PROCESSING,
    /**
     * Signifies that there is no alert.
     */
    CLOSED
};

/**
 * @ingroup microcontroller_proxy
 * 
 * A mutex with protected alert status.
 */
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