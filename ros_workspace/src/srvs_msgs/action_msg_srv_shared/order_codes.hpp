#ifndef ORDER_CODES_HPP
#define ORDER_CODES_HPP

enum OrderCodes {
    MOVE,
    ROTATE,

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
    RELEASE_PUMP
};

#endif
