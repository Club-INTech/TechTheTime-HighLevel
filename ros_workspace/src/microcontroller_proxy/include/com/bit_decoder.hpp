#pragma once
#include <type_traits>
#include <utility>
#include <cstdint>
#include <array>

/**
 * @ingroup microcontroller_proxy
 * 
 * bit_decoder namespace contains bit_decoder::values struct, which allows value's decoding based on pre-defined sheme.
 * It uses meta-function helper bit_decoder::res_decode with a spesialization.
 */
namespace bit_decoder {

    /**
     * Basically res_decode uses trivial encoding sheme, where the data bytes of type To are contained in
     * a variable of type From with a bigger byte size.
     * 
     * The result of decoding is contained in decoded array.
     * 
     * It is sufficient to call decode to fill decoded array.
     * 
     * It has a particular behaviour when [From=uint64 and To=int32] due to the
     * communication particularity. To summarize, in this case the numbers are encoded on 20 bytes
     * but contained in int32 type.
     * 
     * @tparam From type that was used to encode data
     * @tparam To type of data
     */
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

    /**
     * Deduces a return type type of Ftor and uses it as an encoded type.
     * Uses bit_decoder::res_decode to deal with decoding.
     * 
     * A classical snippet of code to use:
     * ```
     *  bit_decoder::values<Func, T> decoded_values{};
     *  auto value = Func(...);
     *  decoded_values.decoder.decode(value);
     *  T example = decoded_values.decoder.decoded.at(i);
     * ```
     * @tparam Ftor a functor that returns encoded value. 
     * @tparam To a type to convert to.
     * @tparam Args arguments taken by functor.
     */
    template<auto& Ftor, typename To, typename... Args>
    struct values {
        using From = typename std::result_of<decltype(Ftor)(Args...)>::type;
        struct res_decode<From, To> decoder{};
    };



};
