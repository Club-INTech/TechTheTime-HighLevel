#ifndef ORDER_BINDER_HPP
#define ORDER_BINDER_HPP

#include <unordered_map>
#include <functional>
#include <cstdint>

template<typename Treq, typename Tres>
struct OrderBinder {

    using wrapped_func_T = typename std::function<void(Treq, Tres)>;

    std::unordered_map<int64_t, wrapped_func_T> orders;

    OrderBinder() : orders() {}

    constexpr void bind_order(int64_t order_code, wrapped_func_T order) {
        orders.insert({order_code, order});
    }

    void execute_order(int64_t order_code, Treq req, Tres res) {
        (orders.find(order_code)->second)(req, res);
    }
};

#endif