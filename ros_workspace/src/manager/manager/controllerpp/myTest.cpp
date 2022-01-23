#include "joystick.hh"
#include "controllerSetup.hpp"
#include <unistd.h>
#include <iostream>

using namespace std;

void test(int button, bool state) {
    cout << "Button " << button << " is " << (state ? "up" : "down") << endl;
}


int main(void) {
    Binder binder;

    binder.BindButton(0, test);
    binder.BindButton(1, test);
    binder.BindButton(2, test);
    binder.BindButton(3, test);
    binder.BindButton(4, test);
    binder.BindButton(5, test);
    binder.BindButton(6, test);
    binder.BindButton(7, test);

    ControllerSetup controller = ControllerSetup(binder);

    std::cout << "Launching" << std::endl;
    controller.run(false);
}