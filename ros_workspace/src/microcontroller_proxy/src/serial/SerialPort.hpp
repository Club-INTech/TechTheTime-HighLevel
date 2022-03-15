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

#include <upd/format.hpp>
#include <rpc/def.hpp>
#include <rpc/master.hpp>
#include <k2o/dispatcher.hpp>

/**
 * @ingroup microcontroller_proxy
 * 
 * A namespace scom is a shortcut for serial communication.
 * It introduces a scom::BlockingModes, scom::SerialPort and other members, that are required
 * for using <a target="_blank" href="https://github.com/Club-INTech/TechTheTime-Shared">TechTheTime-Shared</a> 
 * and <a target="_blank" href="https://github.com/StarQTius/Key-To-Order">Key-To-Order</a>
 * 
 * This libraries allow to communicate with microcontroller.
 * scom::SerialPort introduces some serial communication basics (using termios) and some specific
 * methods used as communication basis of <b>Key-To-Order</b>
 * 
 * @author sudogauss
*/ 
namespace scom {

    /**
     * scom::stuffing_byte is used to indicate a new order transmission  
    */ 
    extern uint8_t stuffing_byte;
    // extern k2o::dispatcher dispatcher;

    /**
     * @ingroup microcontroller_proxy
     * scom::BlockingModes are used to determine which read mode will be used in serial communication
    */ 
    enum class BlockingModes {
        /**
         * Returns immediately with all available data
        */ 
        NO_BLOCK,
        /**
         * Returns the data after the exact required number of bytes has been received 
        */ 
        FIXED_BYTES,
        /**
         * Returns after data is available or after timeout has exceeded
        */ 
        TIMEOUT,
        /**
         * Returns the data after the exact required number of bytes has been received or timeout has exceeded  
        */ 
        TIMEOUT_WITH_FIXED_BYTES
    };

    /**
     * Contains the bytes that are used to configure serial communication control modes 
     * (PARENB, CSTOPB, CRTSCTS, CLOCAL, CREAD) 
    */ 
    extern unsigned int control_mode_bytes[];
    /**
     * Contains the bytes that are used to configure serial communication local modes 
     * (ICANON, ECHO, ECHOE, ECHONL, ISIG) 
    */ 
    extern unsigned int local_modes_bytes[];
    /**
     * Contains the bytes that are used to configure serial communication input modes 
     * (IXON, IXOFF, IXANY, IGNBRK, BRKINT, PARMRK, ISTRIP, INLCR, IGNCR, ICRNL) 
    */ 
    extern unsigned int input_modes_bytes[];
    /**
     * Contains the bytes that are used to configure serial communication output modes 
     * (OPOST, ONLCR) 
    */ 
    extern unsigned int output_modes_bytes[];


    /**
     * @ingroup microcontroller_proxy
     * Class responsible for serial communication, that introduces some special methods to send orders to the
     * microcontroller using <a target="_blank" href="https://github.com/StarQTius/Key-To-Order">Key-To-Order</a>
     * 
     * Inspired a lot by  <a target="_blank" href="https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/">Embedded Ninja</a>
     * 
     * @author sudogauss 
    */
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

        void write_byte(uint8_t*);
        void write_word(uint8_t*, int);

        void read_byte(uint8_t*);
        void read_word(uint8_t*, int);

        void close_port();
        void set_exclusive_access();

        void set_default_config();

        template<auto& Ftor, typename... Args>
        void call_remote_function(Args&&... args) {
            auto key = rpc::master::keyring.get<Ftor>();
            this->com_start_frame_transmission(rpc::Frame_Type::REQUEST);
            key(std::forward<Args>(args)...) >> [&](upd::byte_t byte){this->com_write_byte(byte);};
        }

    private:
        int serial_port = -1;
        const char* port_name = nullptr;
        struct termios serial_port_config;
        size_t write_stuff_counter;

        void com_write_byte(upd::byte_t byte);
        void com_start_frame_transmission(rpc::Frame_Type);
    };

}

/** @} */

#endif