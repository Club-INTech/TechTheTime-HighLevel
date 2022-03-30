#pragma once

#include <tuple>
#include <action_msg_srv_shared/order_codes.hpp>
#include <map>

namespace order_reader {

    extern std::map<std::string, OrderCodes> str_to_order_code;

    inline std::tuple<OrderCodes, double, int64_t, double> get_order_as_tuple(std::string str) {

        OrderCodes order_code;
        double distance;
        int64_t id;
        double angle;

        str.append(" ");

        int processed = 0;
        int prev_str_ptr = 0;
        for(size_t i = 0; i < str.size(); i++) {
            if(str.at(i) == ' ') {
                if(processed == 0) {
                    order_code = str_to_order_code.at(str.substr(prev_str_ptr, i-prev_str_ptr));
                } else if(processed == 1) {
                    distance = std::stod(str.substr(prev_str_ptr, i-prev_str_ptr));
                } else if(processed == 2) {
                    id = std::stoi(str.substr(prev_str_ptr, i-prev_str_ptr));
                } else if(processed == 3) {
                    angle = std::stod(str.substr(prev_str_ptr, i-prev_str_ptr));
                } else {
                    break;
                }
                processed++;
                prev_str_ptr = i + 1;
            }
        }
        return std::make_tuple(order_code, distance, id, angle);
    }
}