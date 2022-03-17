#ifndef ORDER_CODES_HPP
#define ORDER_CODES_HPP

/**
 * OrderCodes are used to distinguish orders and send them from manager to microcontroller_proxy. 
*/ 
enum OrderCodes {
    MOVE,
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
