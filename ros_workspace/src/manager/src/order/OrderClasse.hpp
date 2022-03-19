#include "../client/ClientT.hpp"
#include "../controller/controllerSetup.hpp"
#include "action_msg_srv/srv/order.hpp"
#include "order_codes.hpp"

class Order{
        auto commClient_MCU;
    public:
        Order();
        bool move(double,double);
        bool angle(double);
        bool take_statue(); // On admet qu'il y a pas de recalage à faire
        bool take_palet(int,bool,bool);
};