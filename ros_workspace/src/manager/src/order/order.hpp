#pragma once

#include "../client/ClientT.hpp"
#include "../controller/controllerSetup.hpp"
#include "action_msg_srv/srv/order.hpp"
#include "order_codes.hpp"

namespace order {
    /**
     * @brief Moves
     * 
     * @param distance distance in mm (moves backward if negative)
     * @return Robot successfully moved
     */
    bool move(int distance) {
        return true;
    }

    /**
     * @brief Turns the robot
     *
     * @param angle the relative angle the robot will move to
     * @return Robot successfully moved
     */
    bool turn(int angle) {
        return true;
    }

    /**
     * @brief Stops the robot
     *
     * @return Robot successfully stoped
     */
    bool stop() {
        return true;
    }

    /**
     * @brief Get the Sick Distance object
     * 
     * @param id The id of the sick
     * @return The distance read by the sick
     */
    int getSickDistance(unsigned short id) {
        return 0;
    }

    /**
     * @brief Get the Resistance read by the robot
     * 
     * @return int the ohm value of the resistance
     */
    int getResistance() {
        return 0;
    }

    /**
     * @brief Activate pump to catch a puck in front of id'th arm
     *
     * @param id the id of the arm
     * @return true if puck was successfully caught
     */
    bool catchPuck(unsigned int id) {
        return true;
    }

    /**
     * @brief Deactivate pump to catch a puck in front of id'th arm
     *
     * @param id the id of the arm
     * @return Puck was successfully released
     */
    bool releasePuck(unsigned int id) {
        return true;
    }

    /**
     * @brief Moves the dynamixel
     * 
     * @param id id of the arm, first Dyxel if least than 6, second if more.
     * @param angle the wanted angle of the arm
     * @return The arm could successfully move
     */
    bool moveArm(unsigned int id, int angle) {
        return true;
    }
}