#include "order.hpp"

#define MONTHLERY
#ifdef MONTHLERY

order::Controlls::move(int axis, int value) {
    if (value > 1000 && axis == 1) {
        moveBackward();
    }
    else if (value > 1000 && axis == 0) {
        moveRight();
    }
    else if (value < -1000 && axis == 1) {
        moveForward();
    }
    else if (value < -1000 && axis == 0) {
        moveLeft();
    }
    else {
        stop();
    }
}

order::Controlls::moveArm(int button, bool state) {
    if (state) {
        if (button == upButton) {
            moveArmUp();
        }
        else if (button == downButton) {
            moveArmDown();
        }
    }
}

order::Controlls::changeArm(int button, bool state) {
    if (state) {
        if (button == leftArmButton) {
            armId++;
        }
        else if (button == rightArmButton) {
            armId--;
        }
    }
}

order::Controlls::changePump(int button, bool state) {
    if (state) {
        if (button == activateButton) {
            activatePump();
        }
        else if (button == releaseButton) {
            releasePump();
        }
    }
}

order::Controlls::bindAll(Binder binder) {
    binder.BindAxis(vertAxis, move);
    binder.BindAxis(horAxis, move);

    binder.BindButton(upButton, moveArm);
    binder.BindButton(downButton, moveArm);

    binder.BindButton(leftArmButton, changeArm);
    binder.BindButton(rightArmButton, changeArm);

    binder.BindButton(activateButton, changePump);
    binder.BindButton(releaseButton, changePump);
}

#endif