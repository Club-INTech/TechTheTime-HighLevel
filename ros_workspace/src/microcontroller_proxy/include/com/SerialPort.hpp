#ifndef SCOM_SERIAL_PORT_HPP
#define SCOM_SERIAL_PORT_HPP

#include <cstdio>
#include <string>

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

#include <chrono>
#include <thread>
#include <const_shared/CommunicationConst.hpp>

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
     * scom::stuffing_byte separates data to distinguish data bytes and header bytes.
     * A header is composed of 3 0xff bytes: "0xff 0xff 0xff".
     * If two 0xff bytes occur in data, we must separate them from the third to prevent header repeating,
     * for example: "... 0xff 0xff 0x00 0xff ..."
    */ 
    extern uint8_t stuffing_byte; 

    /**
     * @ingroup microcontroller_proxy
     * scom::BlockingModes are used to determine which read mode will be used in serial communication.
     * It can only be used in non-canonical mode(don't use it if you open serial with O_NDELAY, O_NON_BLOCK, or
     * if you use fcntl)
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
     * @ingroup microcontroller_proxy
     * Class responsible for serial communication, that introduces some special methods to send orders to the
     * microcontroller using <a target="_blank" href="https://github.com/StarQTius/Key-To-Order">Key-To-Order</a>
     * 
     * Inspired a lot by  <a target="_blank" href="https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/">Embedded Ninja</a>
     * 
     * Take a look to <a target="_blank" href="http://manpagesfr.free.fr/man/man3/termios.3.html">termios</a> 
     * for more information about serial communication.
     * 
     * @author sudogauss 
    */
    class SerialPort {

    public:

        /**
         * Basic constructor, which initializes scom::SerialPort::port_name and stuff counters
        */ 
        SerialPort(const char*);

        /**
         * Default destructor 
        */ 
        ~SerialPort() = default;

        /**
         * Opens a serial port or exits with error if an error has occured
        */
        void open_serial();
        
        /**
         * Sets scom::BlockingModes for scom::SerialPort::serial_port_config.
         * It also takes an initializer_list of arguments and checks its length.
         * You can provide from 0 to 2 arguments (timeout and bytes number to wait).
         * 
         * @param mode a blocking mode.
         * @param args an initializer list of arguments <b>for a particular blocking mode</b>
        */ 
        void define_blocking_mode(BlockingModes mode, std::initializer_list<int> args);

        /**
         * Writes a byte to serial
         * @param byte a byte to write
        */ 
        void write_byte(uint8_t* byte);

        /**
         * Writes a word to serial
         * @param word byte sequence to write
         * @param size the size of byte sequence to write
        */ 
        void write_word(uint8_t* word, int size);

        /**
         * Read a byte from serial with respect to the scom::BlockingModes
         * @param byte an address of byte to read into
        */ 
        void read_byte(uint8_t* byte);

        /**
         * Read a sequence of bytes from serial with respect to the scom::BlockingModes
         * @param word an address of word to read into
         * @param size a size of word to read
        */ 
        void read_word(uint8_t* word, int size);

        /**
         * Closes serial port
         */ 
        void close_port();

        /**
         * Sets an exclusive to the serial port by the current process.
         * @throws std::runtime_error if port is already in use. 
        */ 
        void set_exclusive_access();

        /**
         * Flushes the input system buffer.
         * Mainly used to avoid the data collision as multiple threads or processes use the same port.
         */
        void flush();

        /**
         * Opens serial, gets config, then sets a default config to serial port.
         * The default config is the following:
         * <ul>
         * <li>115200 bauds for input an output speed</li>
         * <li>TIMEOUT blocking mode with 0.5s timeout</li>
         * <li>8 bits per byte</li>
         * <li>control modes: {false, false, false, true, true}</li>
         * <li>local modes: {false, false, false, false, false}</li>
         * <li>input modes: {false, false, false, false, false, false, false, false, false, false}</li>
         * <li>output modes: {false, false}</li>
         * </ul>
         * 
         * It operates in non-blocking mode, non-canonical mode and return data byte per byte as they are
         * available.
        */ 
        void set_default_config();

        void flush();

        /**
         * A generic rpc function to call another function(an order) on the target device. 
         * It gets a key, based on the provided functor, from a master keyring.
         * 
         * For more information on keyring, see <a target="_blank" href="https://github.com/StarQTius/Key-To-Order">Key-To-Order</a>
         * 
         * @tparam Ftor rpc functor(a function we want to call on remote device) 
         * @tparam Args arguments type needed to call remote function on target device
         * 
         * @param args arguments to call remote function
        */ 
        template<auto& Ftor, typename... Args>
        void call_remote_function(Args&&... args) {
            auto key = rpc::master::keyring.get<Ftor>();
            this->com_start_frame_transmission(rpc::Frame_Type::REQUEST);
            key(std::forward<Args>(args)...) >> [&](upd::byte_t byte){this->com_write_byte(byte);};
            std::this_thread::sleep_for(SERIAL_COM_DELAY);
        }

        /**
         * A generic rpc function that receives a feedback after Ftor call on remote device.
         * It gets a key, based on the provided functor, from a master keyring.
         * 
         * For more information on keyring, see <a target="_blank" href="https://github.com/StarQTius/Key-To-Order">Key-To-Order</a>
         * 
         * @tparam Ftor rpc functor(a function we want to call on remote device)
         * @return data of deduced type
         */
        template<auto& Ftor>
        auto receive_feedback() {
            this->new_response = true;
            auto key = rpc::master::keyring.get<Ftor>();
            std::this_thread::sleep_for(READ_FEEDBACK_DELAY);
            return key << [&]() {return this->com_read_byte();};
        }

    private:
        /**
         * Serial port id(descriptor).
         * The default value is -1, because the port has not been opened.
        */ 
        int serial_port = -1;

        /**
         * Port name (example: /dev/ttyACM0).
         * The default value is nullptr as scom::SerialPort is not initialized.
        */ 
        const char* port_name = nullptr;

        /**
         * A write counter used to distinguish frame header bytes and data bytes in writing. 
         * Initiates a stuffing byte transmission.
         * Take a look at scom::stuffing_byte for explanation.
        */ 
        size_t write_stuff_counter;

        /**
         * A read counter used to distinguish frame header bytes and data bytes in reading.
         * Indicates whether the next byte is stuffing or a data byte.
         * Take a look at scom::stuffing_byte for explanation.
         */
        size_t read_stuff_counter;

        /**
         * Triggers to true as new response must be read.
         * Needed to know whether a frame header is expected or not.
         */
        bool new_response;

        /**
         * Writes a byte to the serial port and takes care of scom::SerialPort::write_stuff_counter
         * and separation of header and data.
         * 
         * Passed in key of <a target="_blank" href="https://github.com/StarQTius/Key-To-Order">Key-To-Order</a> 
         * as output method.
         * 
         * @param byte a byte to send.
        */ 
        void com_write_byte(upd::byte_t byte);

        /**
         * Sends a sequence of header bytes that determine the frame type(message type).
         * To know more about frame types you may refer to the 
         * <a target="_blank" href="https://github.com/Club-INTech/TechTheTime-Shared/blob/master/rpc/def.hpp">TechTheTime-Shared</a>.
         * 
         * For example: 0x00 0xff 0xff 0xff 0x01 for response.
         *               ^    ^    ^    ^    ^
         *               |    |    |    |    |
         *          stuffing     header   frame_type
         * 
         * @param frame_type a type of the frame(of the message)
        */ 
        void com_start_frame_transmission(rpc::Frame_Type frame_type);

        /**
         * Read a byte from the serial port accordng to the defined protocol.
         * The header is read and ignored.
         * Takes care of stuffing bytes.
         * 
         * @return upd::byte_t a byte that has been read 
         */
        upd::byte_t com_read_byte();

    }; 

}

/** @} */

#endif