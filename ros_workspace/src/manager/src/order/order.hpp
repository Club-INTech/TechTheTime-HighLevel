#include "../client/ClientT.hpp"
#include "../controller/controllerSetup.hpp"
#include "action_msg_srv/srv/order.hpp"
#include "order_codes.hpp"

class Order{
        auto commClient;
    public:
        Order();
        bool move(double,double);
        bool moveABS(double);
        bool angleABS(double);
        bool take_statue();
        bool drop_replic();
        bool take_palet_distrib(int);
};