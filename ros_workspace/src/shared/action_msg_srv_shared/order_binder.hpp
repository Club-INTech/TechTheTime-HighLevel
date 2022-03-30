#ifndef ORDER_BINDER_HPP
#define ORDER_BINDER_HPP

#include <unordered_map>
#include <functional>
#include <cstdint>
#include <stdexcept>

/**
 * OrderBinder is used by ActionService in order to execute callback attached to an order code.
 * You can bind any of OrderCodes(order_codes.hpp) to any function with the OrderBinder::order_functor_T signature and then execute
 * it by providing an <b>order_code</b> and parameters.
 * 
 * @tparam Treq The type of request for the order
 * @tparam Tres The type of response of an order 
*/ 
template<typename Treq, typename Tres>
struct OrderBinder {

    /**
     * An alias to the function type that can be associated to one of the OrderCodes(order_codes.hpp) 
    */ 
    using order_functor_T = typename std::function<void(Treq, Tres)>;

    /**
     * A map that associate an <b>order_code</b> to the callback
    */ 
    std::unordered_map<int64_t, order_functor_T> orders;

    /**
     * Default constructor, calls a default constructor of OrderBinder::orders 
    */ 
    OrderBinder() : orders() {}

    /**
     * Associates an order(an OrderBinder::order_functor_T) to an order_code
     * @param order_code A code of an order (type is int64_t to allow a direct execution of an associated order 
     * after receiving it from the ClientT)
     * @param order A functor associated to the <b>order_code</b> 
    */ 
    constexpr void bind_order(int64_t order_code, order_functor_T order) {
        orders.insert({order_code, order});
    }

    /**
     * Executes an order(an OrderBinder::order_functor_T) associated to an <b>order_code</b>
     * 
     * @param order_code A code of an order (type is int64_t to allow a direct execution of an associated order 
     * after receiving it from the ClientT)
     * @param req A received request, that must be treated
     * @param res A response that must be composed and sended back
    */ 
    void execute_order(int64_t order_code, Treq req, Tres res) {
        if(orders.find(order_code) == orders.end()) throw std::runtime_error("Trying to access a order with unknown code");
        (orders.find(order_code)->second)(req, res);
    }
};

#endif