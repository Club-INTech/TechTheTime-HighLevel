#include "order.hpp"

namespace order {
#ifdef MONTHLERY
    // void move(int axis, int value) {
    //     if (value > 1000 && axis == 1) {
    //         moveBackward();
    //     }
    //     else if (value > 1000 && axis == 0) {
    //         rotateRight();
    //     }
    //     else if (value < -1000 && axis == 1) {
    //         moveForward();
    //     }
    //     else if (value < -1000 && axis == 0) {
    //         rotateLeft();
    //     }
    //     else {
    //         stop();
    //     }
    // }

    // void moveArm(int button, bool state) {
    //     if (state) {
    //         if (button == upButton) {
    //             moveArmUp();
    //         }
    //         else if (button == downButton) {
    //             moveArmDown();
    //         }
    //     }
    // }

    // void changeArm(int button, bool state) {
    //     if (state) {
    //         if (button == leftArmButton) {
    //             armId++;
    //         }
    //         else if (button == rightArmButton) {
    //             armId--;
    //         }
    //     }
    // }

    // void changePump(int button, bool state) {
    //     if (state) {
    //         if (button == activateButton) {
    //             activatePump();
    //         }
    //         else if (button == releaseButton) {
    //             releasePump();
    //         }
    //     }
    // }

    // void bindAll(Binder binder) {
    //     binder.BindAxis(vertAxis, move);
    //     binder.BindAxis(horAxis, move);

    //     binder.BindButton(upButton, moveArm);
    //     binder.BindButton(downButton, moveArm);

    //     binder.BindButton(leftArmButton, changeArm);
    //     binder.BindButton(rightArmButton, changeArm);

    //     binder.BindButton(activateButton, changePump);
    //     binder.BindButton(releaseButton, changePump);
    // }
#else

    bool goTo(int x, int y)
    {
        std::cout << "Goes to " << x << ", " << y << "." << std::endl;

        return true;
    }

    bool moveForward()
    {
        std::cout << "Is moving forward" << std::endl;
        return true;
    }
    bool moveBackward()
    {
        std::cout << "Is moving backward" << std::endl;
        return true;
    }
    bool moveLeft()
    {
        std::cout << "Is turning left" << std::endl;
        return true;
    }
    bool moveRight()
    {
        std::cout << "Is turning right" << std::endl;
        return true;
    }

    void joystickMove(int axis, int value)
    {
        if (value > 1000 && axis == 1)
        {
            moveBackward();
        }
        else if (value > 1000 && axis == 0)
        {
            moveRight();
        }
        else if (value < -1000 && axis == 1)
        {
            moveForward();
        }
        else if (value < -1000 && axis == 0)
        {
            moveLeft();
        }
    }

    bool takeAA()
    {
        return true;
    }

#endif
}