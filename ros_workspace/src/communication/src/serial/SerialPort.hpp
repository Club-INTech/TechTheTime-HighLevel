#ifndef SCOM_SERIAL_PORT_HPP
#define SCOM_SERIAL_PORT_HPP

#include <cstdio>
#include <string>

#include <fcntl.h> 
#include <errno.h> 
#include <termios.h> 
#include <unistd.h>
#include <iostream>

#include <initializer_list>

#include <cstddef>
namespace scom {

    enum class BlockingModes {
        NO_BLOCK,
        FIXED_BYTES,
        TIMEOUT,
        TIMEOUT_WITH_FIXED_BYTES
    };

    extern unsigned int control_mode_bytes[];
    extern unsigned int local_modes_bytes[];
    extern unsigned int input_modes_bytes[];
    extern unsigned int output_modes_bytes[];


    class SerialPort {

    public:
        SerialPort(const char*);
        ~SerialPort() = default;


        void open_serial();
        void set_config();
        void get_config();

        void set_input_speed(speed_t);
        void set_output_speed(speed_t);

        void define_blocking_mode(BlockingModes, std::initializer_list<int>);

        void configure_control_modes(tcflag_t, std::initializer_list<bool>);
        void configure_local_modes(std::initializer_list<bool>);
        void configure_input_modes(std::initializer_list<bool>);
        void configure_output_modes(std::initializer_list<bool>);

        ssize_t write_byte(uint8_t);
        ssize_t read_word(uint8_t*, int);

        ssize_t read_byte(uint8_t*);

        void close_port();
        void set_exclusive_access();

        void set_default_config();

    private:
        int serial_port = -1;
        const char* port_name = nullptr;
        struct termios serial_port_config;  

    };

}

#endif