#pragma once

#include "../client/ActionClient.hpp"
#include "action_msg_srv/srv/order.hpp"
#include "order_codes.hpp"

class Order{
    public:
        Order();
        bool move(double, double);
        bool moveABS(double);
        bool angleABS(double);
        bool take_statue();
        bool drop_replic();
        bool take_palet_distrib(int);
    private:
        auto commClient;
};