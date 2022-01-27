#include <iostream>
#include <cmath>

namespace order {

    bool isMovingForward = false;
    bool isMovingBackward = false;

    bool isTurningRighthand = false;
    bool isTurningLefthand = false;

    bool goTo(int x, int y) {
        std::cout << "Goes to " << x << ", " << y << "." << std::endl;

        return true;
    }

    bool moveForward() {
        std::cout << "Is moving forward" << std::endl;
        return true;
    }
    bool moveBackward() {
        std::cout << "Is moving backward" << std::endl;
        return true;
    }
    bool moveLeft() {
        std::cout << "Is turning left" << std::endl;
        return true;
    }
    bool moveRight()
    {
        std::cout << "Is turning right" << std::endl;
        return true;
    }

    void joystickMove(int axis, int value) {
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
    }

    bool takeAA() {
        return true;
    }
}