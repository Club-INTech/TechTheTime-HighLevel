#include "../joystick/joystick.hpp"
#include "../controllerSetup.hpp"
#include <unistd.h>
#include <iostream>

using namespace std;

void test(int button, bool state) {
    cout << "Button " << button << " is " << (state ? "up" : "down") << endl;
}
void testA(int axis, float value) {
    cout << "Axis " << axis << " is " << value << endl;
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
    binder.BindButton(8, test);
    binder.BindButton(9, test);
    binder.BindButton(10, test);
    binder.BindAxis(0, test);
    binder.BindAxis(1, test);
    binder.BindAxis(2, test);
    binder.BindAxis(3, test);
    binder.BindAxis(4, test);
    binder.BindAxis(5, test);
    binder.BindAxis(6, test);

    ControllerSetup controller = ControllerSetup(binder);

    std::cout << "Launching" << std::endl;
    controller.run(true);
}