#include "../client/ClientT.hpp"
#include "../controller/controllerSetup.hpp"
#include "action_msg_srv/srv/order.hpp"
#include "order_codes.hpp"

class Order{
        auto commClient;
    public:
        Order();
        bool move(double,double);
        bool angle(double);
        bool take_statue();
        bool drop_replic();
        bool take_distrib_vertical(int); // Take the id of the distrib
        bool take_distrib_horizon(int); // Take the id of the distrib
};