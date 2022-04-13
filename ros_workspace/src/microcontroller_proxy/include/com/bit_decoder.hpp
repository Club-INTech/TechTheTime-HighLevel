#pragma once
#include <type_traits>
#include <utility>
#include <cstdint>
#include <array>

namespace bit_encoder {

    template<typename From, typename To>
    struct res_decode {
        using from_type = From;
        using to_type = To;

        bool is_possible = (sizeof(from_type) >= sizeof(to_type)); 

        std::array<to_type, (int) (sizeof(from_type) / sizeof(to_type))> decoded = {};

        void decode(from_type encoded) {
            int counter_ = 0;
            for(auto it = decoded.cbegin(); it < decoded.cend(); it++) {
                *it = (encoded >> counter_);
                counter_ += (8 * sizeof(to_type));
            }
        }
    };

    template<>
    struct res_decode<int32_t, int32_t> {

        bool is_possible = true;

        std::array<int32_t, 1> decoded = {};
        void decode(int32_t encoded) {
            auto it = decoded.begin();
            if(encoded & ((int32_t) 1 << 20)) {
                *it = -(encoded & (((int32_t) 1 << 20)-1));
            } else {
                *it = encoded;
            }
        } 
    };

    template<>
    struct res_decode<uint64_t, int32_t> {

        bool is_possible = true;

        std::array<int32_t, 2> decoded = {};
        void decode(uint64_t encoded) {
            auto it = decoded.begin();

            int32_t first = (int32_t) (encoded >> 32);
            int32_t second = (int32_t) (encoded & ((int32_t) ((1 << 32) - 1)));

            if(first & ((int32_t) 1 << 20)) {
                *it = -(first & (((int32_t) 1 << 20)-1));
            } else {
                *it = first;
            }

            it++;

            if(second & ((int32_t) 1 << 20)) {
                *it = -(second & (((int32_t) 1 << 20)-1));
            } else {
                *it = second;
            }
        } 
    };

    template<auto& Ftor, typename To, typename... Args>
    struct values {
        using From = typename std::result_of<decltype(Ftor)(Args...)>::type;
        struct res_decode<From, To> decoder{};
    };



};
