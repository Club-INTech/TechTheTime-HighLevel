#ifndef ORDER_CODES_HPP
#define ORDER_CODES_HPP

/**
 * OrderCodes are used to distinguish orders and send them from manager to microcontroller_proxy. 
*/ 
enum OrderCodes {
    MOVE,
    ///Julien : idées mais on peut changer
    //Avance tout droit
    START_MOVE_FORWARD,
    //Recule tout droit
    START_MOVE_BACKWARD,
    //Tourne à gauche
    START_ROTATE_LEFT,
    //Tourne à droite
    START_ROTATE_RIGHT,
    //Arrête tous les mvts (des roues)
    STOP,

    //Bouge le bras d'index id à l'angle angle.
    MOVE_ARM,
    //Active la pompe d'index id
    ACTIVATE_PUMP,
    //Relache la pompe d'index id
    RELEASE_PUMP,
    ROTATE
};

/**
 * MotionStatusCodes are used to determine the status of the robot after movement 
*/ 
enum MotionStatusCodes {
    /**
     * MOTION_TIMEOUT indicates that the requested motion has not been done in required time interval  
    */ 
    MOTION_TIMEOUT,
    /**
     * NOT_COMPLETE indicates that the robot stopped without completing the movement 
    */ 
    NOT_COMPLETE,
    /**
     * COMPLETE indicates that the robot completed movement and stopped 
    */ 
    COMPLETE
};

#endif
