#include "joystick/joystick.cpp"
#include "controllerSetup.hpp"

ControllerSetup::ControllerSetup(Binder binder) {
    Joystick joystick = Joystick("/dev/input/js0");
    if (!joystick.isFound())
    {
        printf("joystick not found.\n");
        exit(1);
    }
    this->binder = binder;
}

ControllerSetup::ControllerSetup(std::string path, Binder binder)
{
    Joystick joystick = Joystick(path);
    if (!joystick.isFound())
    {
        printf("joystick not found.\n");
        exit(1);
    }
    this->binder = binder;
}

void ControllerSetup::run(bool debug) {
    while (true) {
        usleep(1000);

        JoystickEvent event;
        if (joystick.sample(&event))
        {
            if (event.isButton())
            {
                bool state = event.value == 0 ? true : false;
                binder.CallButtonCallback(event.number, state);

                if (debug) {
                    printf("Button %u is %s\n",
                        event.number,
                        state ? "up" : "down");
                }
            }
            else if (event.isAxis())
            {
                binder.CallAxisCallback(event.number, event.value);

                if (debug) {
                    printf("Axis %u is at position %d\n", event.number, event.value);
                }
            }
        }
    }
}