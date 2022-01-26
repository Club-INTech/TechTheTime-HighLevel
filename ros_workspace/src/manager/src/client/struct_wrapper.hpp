#ifndef STRUCT_INIT_HPP
#define STRUCT_INIT_HPP

template<typename T, typename... Ts>
struct struct_wrapper{
    using type = T;

    std::shared_ptr<T> value;

    void set_values(Ts... args) {
        T tmp = {args...};
        *value = tmp;
    }
};

#endif