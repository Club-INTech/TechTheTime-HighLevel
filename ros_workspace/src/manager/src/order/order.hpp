#include "../client/ClientT.hpp"
#include "../controller/controllerSetup.hpp"
#include "action_msg_srv/srv/order.hpp"
#include "order_codes.hpp"

namespace order {
#define MONTHLERY
#ifdef MONTHLERY

    class Controlls
    {
    private:
        const int vertAxis = 0;
        const int horAxis = 1;

        const int upButton = 4;
        const int downButton = 5;
        const int leftArmButton = 1;
        const int rightArmButton = 3;
        const int activateButton = 0;
        const int releaseButton = 2;

    public:
        std::shared_ptr<ClientT<action_msg_srv::srv::Order, action_msg_srv::srv::Order::Request, int64_t, int64_t, int64_t, int64_t>> client_ptr;

        int64_t armId = 0;
        std::array<int, 6> armsAngle = {0, 0, 0, 0, 0, 0}; //Angles par défauts

        const int step = 10;

        // bool isMovingForward = false;
        // bool isMovingBackward = false;

        // bool isTurningRighthand = false;
        // bool isTurningLefthand = false;

        Controlls(std::shared_ptr<ClientT<action_msg_srv::srv::Order, action_msg_srv::srv::Order::Request, int64_t, int64_t, int64_t, int64_t>> client_ptr);

        void move(int axis, int value);

        void moveArm(int axis, int value);

        void changeArm(int button, int state);

        void changePump(int button, int state);

        void moveForward() {
            client_ptr->send(OrderCodes::START_MOVE_FORWARD, 0, 0, 0);
        }

        void moveBackward() {
            client_ptr->send(OrderCodes::START_MOVE_BACKWARD, 0, 0, 0);
        }

        void rotateLeft() {
            client_ptr->send(OrderCodes::START_ROTATE_LEFT, 0, 0, 0);
        }

        void rotateRight() {
            client_ptr->send(OrderCodes::START_ROTATE_RIGHT, 0, 0, 0);
        }

        void stop() {
            client_ptr->send(OrderCodes::STOP, 0, 0, 0);
        }

        void moveArmUp() {
            armsAngle[armId] += step;
            client_ptr->send(OrderCodes::MOVE_ARM, 0, armId, armsAngle[armId]);
        }

        void moveArmDown() {
            armsAngle[armId] -= step;
            client_ptr->send(OrderCodes::MOVE_ARM, 0, armId, armsAngle[armId]);
        }

        void changeArmUp() {
            armsAngle[armId] += step;
            client_ptr->send(OrderCodes::MOVE_ARM, 0, armId, armsAngle[armId]);
        }

        void activatePump() {
            client_ptr->send(OrderCodes::ACTIVATE_PUMP, 0, armId, 0);
        }
        void releasePump() {
            client_ptr->send(OrderCodes::RELEASE_PUMP, 0, armId, 0);
        }

        void bindAll(Binder binder);
    };


#else


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

#endif
}