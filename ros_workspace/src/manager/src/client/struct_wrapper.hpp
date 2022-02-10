#ifndef STRUCT_INIT_HPP
#define STRUCT_INIT_HPP

template<typename T, typename... Ts>
struct struct_wrapper{
    using type = T;

    std::shared_ptr<T> value;

    struct_wrapper() {
        this->value = std::make_shared<T>();
    }

    void set_values(Ts... args) {
        T tmp = T(args...);
        *value = tmp;
    }
};

#endif