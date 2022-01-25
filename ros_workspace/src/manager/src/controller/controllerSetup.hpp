#ifndef __CONTROLLERSETUP_HH__
#define __CONTROLLERSETUP_HH__

#include "joystick/joystick.hpp"
#include <array>
#include <functional>
#include <unistd.h>

class Binder
{
private:
    std::array<std::function<void(int, bool)>, 11> buttonsCallback;
    std::array<std::function<void(int, float)>, 8> axisCallback;

public:
    Binder() {
        buttonsCallback.fill([](int, bool) {});
        axisCallback.fill([](int, float) {});
    }

    /**
     * @brief Binds the button index with the callback function
     * 
     * @param button The button that will be bound
     * @param func The function that will be bound
     */
    void BindButton(int button, std::function<void(int, bool)> func) {
        buttonsCallback[button] = func;
    }

    /**
     * @brief Binds the axis index with the callback function
     * 
     * @param axis The axis that will be bound
     * @param func The function that will be bound
     */
    void BindAxis(int axis, std::function<void(int, float)> func) {
        axisCallback[axis] = func;
    }


    /**
     * @brief Calls the callback assiociated with the button index
     * 
     * @param button The button
     * @param state The state of the button
     */
    void CallButtonCallback(int button, bool state) {
        (buttonsCallback.at(button))(button, state);
    }

    /**
     * @brief Calls the callback assiociated with the axis index
     * 
     * @param axis The axis index
     * @param value The value of the axis
     */
    void CallAxisCallback(int axis, float value) {
        (axisCallback.at(axis))(axis, value);
    }
};

// cs = new ControllerSetup<int, float>
class ControllerSetup
{
public:
    Joystick joystick;
    Binder binder;
    bool isRunning;

    /**
     * @brief Cannot create empty controller
     */
    ControllerSetup() = delete;

    /**
     * @brief Construct a new Controller Setup object
     * 
     * @param binder The binder associated
     */
    ControllerSetup(Binder binder);

    /**
     * @brief Construct a new Controller Setup object
     * 
     * @param path The path of the joystick
     * @param binder The binder associated
     */
    ControllerSetup(std::string path, Binder binder);

    /**
     * @brief Runs the controller with the bound callbacks
     * 
     * @param debug Prints debug if true
     */
    void run(bool debug);
};

#endif