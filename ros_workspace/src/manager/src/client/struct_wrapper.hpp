#ifndef STRUCT_INIT_HPP
#define STRUCT_INIT_HPP

template<typename T, typename... Ts>
struct struct_wrapper{
    using type = T;

    std::shared_ptr<T> value;

    struct_wrapper(std::shared_ptr<T>&& v_ptr) {
        value = std::move(v_ptr);
    }

    void set_values(Ts... args) {
        T tmp = {args...};
        *value = t;
    }



};

#endif