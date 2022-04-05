#include "order_reader.hpp"
#include <iostream>
#include <tuple>

std::map<std::string, OrderCodes> order_reader::str_to_order_code{
    {"move", OrderCodes::MOVE},
    {"rotate_left", OrderCodes::START_ROTATE_LEFT},
    {"rotate_right", OrderCodes::START_ROTATE_RIGHT}
};