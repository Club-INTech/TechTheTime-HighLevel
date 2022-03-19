#pragma once

#include "../client/ClientT.hpp"
#include "../controller/controllerSetup.hpp"
#include "action_msg_srv/srv/order.hpp" //Order.srv ?
#include "order_codes.hpp"

namespace order {
    /**
     * @brief Moves
     * 
     * @param distance distance in mm (moves backward if negative)
     * @return Robot successfully moved
     */
    bool move(int x, int y) { 

        // Traitement feedback response
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
     * @brief Activate pump to catch a puck in front of id'th arm
     *
     * @param id the id of the arm
     * @return true if puck was successfully caught
     */
    bool catchPuck(unsigned int id) {
        return true;
    }

    /**
     * @brief Deactivate pump to catch a puck in front of idth arm
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

    /**
     * @brief Moves the arms and activates the pumps to take the puck to the idth arm from the floor
     * 
     * @param objectiveId The id of the arm at the end
     * @return The action is successfull
     */
    bool takePuckFromFloor(unsigned int objectiveId) {
        return true;
    }

    /**
     * @brief Moves the arms and activates the pumps to exchange the puck - bottom to top if id is top, top to bottom if id is bottom
     *
     * @param objectiveId The id of the arm at the end
     * @return The action is successfull
     */
    bool exchangePuck(unsigned int objectiveId) {
        return true;
    }

    /**
     * @brief Puts down the puck located at the idth arm
     * 
     * @param fromId The id of the arm that should put down the puck
     * @return The action was successfull
     */
    bool putDownPuck(unsigned int fromId) {
        return true;
    }

    /**
     * @brief Activates the stroller of the pucks
     * 
     * @return The action was successfull
     */
    bool activateStroller() {
        return true;
    }

    /**
     * @brief Deactivates the stroller of the pucks
     * 
     * @return true 
     * @return false 
     */
    bool deactivateStroller() {
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

    //bool 
}