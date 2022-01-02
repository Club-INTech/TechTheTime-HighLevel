#ifndef SCOM_SERIAL_PORT_HPP
#define SCOM_SERIAL_PORT_HPP

#include <cstdio>
#include <string>

#include <fcntl.h> 
#include <errno.h> 
#include <termios.h> 
#include <unistd.h>

#include <cinttypes>
namespace scom {

    enum class BlockingModes {
        NO_BLOCK,
        FIXED_BYTES,
        TIMEOUT,
        TIMEOUT_WITH_FIXED_BYTES
    };

    enum class BaudRate {

    };

    class SerialPort {

    public:
        SerialPort() = default;
        SerialPort(const char*);
        ~SerialPort() = default;

        SerialPort(const SerialPort&) = delete;
        SerialPort(const SerialPort&&) = delete;

        void open_serial(bool);
        void set_config();
        void get_config();

        void set_input_speed(speed_t);
        void set_output_speed(speed_t);

        speed_t get_input_speed(speed_t);
        speed_t get_output_speed(speed_t);

        void define_blocking_mode(BlockingModes, u_int8_t...);

        void configure_control_modes(bool, bool, tcflag_t, bool);
        void configure_local_modes(bool, bool, bool);
        void configure_input_modes(bool, bool);
        void configure_output_modes(bool, bool);

        template<typename T>
        void write(T data);

        template<typename T>
        void read(T* buf);

        void close_port();
        void set_exclusive_access();

    private:
        int serial_port = -1;
        const char* port_name = nullptr;
        struct termios2* serial_port_config;  

    };
}

#endif