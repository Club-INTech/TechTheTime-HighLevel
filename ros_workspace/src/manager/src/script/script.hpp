#pragma once

#include "../client/ActionClient.hpp"
#include "action_msg_srv/srv/order.hpp"
#include <action_msg_srv_shared/order_codes.hpp>
#include <functional>
#include <deque>
#include <variant>
#include <unordered_map>
#include <fstream>

class Script{
    public:

        using void_profile_T = std::function<void()>;
        using move_profile_T = std::function<void(double, double)>;
        using actuator_profile_T = std::function<void(double, int)>;
        using id_profile_T = std::function<void(int)>;

        using order_profile_T = std::variant<
            void_profile_T,
            move_profile_T,
            actuator_profile_T,
            id_profile_T
        >;

        Script();

        void parse_script(const char* script_file);

        void wait_for_jumper();
        void move(double, double);
        void moveREL(double, int);
        void angleABS(double, int);

        void take_statue();
        void drop_statue();
        void drop_replic();

        void reverse_palet(int);
        void mesure();
        void knock_over();

        void down_servos();
        void up_servos();

        void take_palet_vertical(int);
        void take_palet_horizontal(int);
        void take_palet_ground(int);

        void drop_palet_gallery(int);
        void drop_palet_ground(int);
        
        void pushOrder(std::function<void()>);
        void run();

        bool treat_response(MotionStatusCodes status, std::function<void()> OrderToReinsert, bool reinsert);

    private:
        std::shared_ptr<ActionClient> commClient;
        std::deque<std::function<void()>> deque_order;
        std::unordered_map<std::string, order_profile_T> orders{};
};