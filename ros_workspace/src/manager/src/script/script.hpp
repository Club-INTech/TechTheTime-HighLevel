#pragma once

#include "../client/ActionClient.hpp"
#include "action_msg_srv/srv/order.hpp"
#include <action_msg_srv_shared/order_codes.hpp>
#include <functional>
#include <deque>
#include <variant>
#include <unordered_map>

class Script{
    public:

        // using void_profile_T = std::function<void()>;
        // using move_profile_T = std::function<void(double, double)>;
        // using actuator_profile_T = std::function<void(double, int)>;
        // using id_profile_T = std::function<void(int)>;

        // using order_profile_T = std::variant<
        //     void_profile_T,
        //     move_profile_T,
        //     actuator_profile_T,
        //     id_profile_T
        // >;

        Script();

        void wait_for_jumper();
        void move(double, double, int);
        void moveABS(double, double, int);
        void angleABS(double, double, int);

        void take_statue(double, double, int);
        void drop_replic(double, double, int);

        void reverse_palet(double, double, int);
        void mesure(double, double, int);
        void knock_over(double, double, int);

        void down_servos(double, double, int);
        void up_servos(double, double, int);

        void take_palet_vertical(double, double, int);
        void take_palet_horizontal(double, double, int);
        void take_palet_ground(double, double, int);

        void drop_palet_gallery(double, double, int);
        void drop_palet_ground(double, double, int);
        
        void pushOrder(std::function<void()>);
        void run();

        bool treat_response(MotionStatusCodes status, std::function<void()> OrderToReinsert, bool reinsert);

    private:
        std::shared_ptr<ActionClient> commClient;
        std::deque<std::function<void()>> deque_order;
        // std::unordered_map<std::string, order_profile_T> orders{};
};