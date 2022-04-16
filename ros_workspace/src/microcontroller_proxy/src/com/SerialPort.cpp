#include <com/SerialPort.hpp>
#include <cerrno>
#include <cstring>
#include <vector>
#include <cassert>
#include <unistd.h>
#include <sys/file.h>
#include <stdexcept>

#include <upd/format.hpp>
#include <rpc/def.hpp>
#include <rpc/master.hpp>
#include <k2o/dispatcher.hpp>

#include "rclcpp/rclcpp.hpp"

using namespace scom;

namespace scom {

    uint8_t stuffing_byte = ~rpc::header[0];
}

SerialPort::SerialPort(const char* port_name) {
    this->port_name = port_name;
    this->write_stuff_counter = 0;
    this->read_stuff_counter = 0;
    this->new_response = false;
}

void SerialPort::open_serial() {
    int serial_port  = open(this->port_name, O_RDWR);
    sleep(1);
    this->flush();
    if (serial_port < 0) {
        printf("Error %i open: %s\n", errno, std::strerror(errno));
        exit(errno);
    }
    this->serial_port = serial_port;
}

void SerialPort::define_blocking_mode(BlockingModes mode, std::initializer_list<int> args) {

    struct termios serial_port_config;

    if(tcgetattr(this->serial_port, &serial_port_config) < 0) {
        printf("Error %i get config: %s\n", errno, std::strerror(errno));
        exit(errno);
    }

    if(mode == BlockingModes::NO_BLOCK) {
        serial_port_config.c_cc[VTIME] = 0;
        serial_port_config.c_cc[VMIN] = 0;
    } else if (mode == BlockingModes::FIXED_BYTES) {
        assert(args.size() == 1);
        serial_port_config.c_cc[VTIME] = 0;
        auto it = args.begin();
        serial_port_config.c_cc[VMIN] = *it;
    } else if (mode == BlockingModes::TIMEOUT) {
        assert(args.size() == 1);
        serial_port_config.c_cc[VMIN] = 0;
        auto it = args.begin();
        serial_port_config.c_cc[VTIME] = *it;
    } else if (mode == BlockingModes::TIMEOUT_WITH_FIXED_BYTES) {
        assert(args.size() == 2);
        auto it = args.begin();
        serial_port_config.c_cc[VTIME] = *it;
        it++;
        serial_port_config.c_cc[VMIN] = *it;
    }
    
    if(tcsetattr(this->serial_port, TCSANOW ,&serial_port_config) < 0) {
        printf("Error %i set config: %s\n", errno, std::strerror(errno));
        exit(errno);
    }

}

void SerialPort::write_byte(uint8_t* byte) {
    if(write(this->serial_port, byte, 1) == -1) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Write byte failure.");
        rclcpp::shutdown();
        exit(1);
    }
}

void SerialPort::write_word(uint8_t* word, int size) {
    if(write(this->serial_port, word, size) == -1) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Write word failure.");
        rclcpp::shutdown();
        exit(1);
    }
}

void SerialPort::read_byte(uint8_t* byte) {
    if(read(this->serial_port, byte, 1) == -1) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Read byte failure.");
        rclcpp::shutdown();
        exit(1);
    }
}

void SerialPort::read_word(uint8_t* word, int size) {
    if(read(this->serial_port, word, size) == -1) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Read word failure. %s", std::strerror(errno));
        rclcpp::shutdown();
        exit(1);
    }
}

void SerialPort::com_write_byte(upd::byte_t byte) {
    if(byte == rpc::header[0]) {
        this->write_stuff_counter++;
    } else {
        this->write_stuff_counter = 0;
    }

    this->write_byte(&byte);

    if(this->write_stuff_counter == sizeof(rpc::header) - 1) {
        this->write_stuff_counter = 0;
        this->write_byte(&stuffing_byte);
    }
}

upd::byte_t SerialPort::com_read_byte() {
    upd::byte_t header[5];
    if(this->new_response) {
        for(int i = 0; i < 5; i++) {
            this->read_byte(&header[i]);
        }
        this->new_response = false;
    }

    upd::byte_t byte; 
    
    if(this->read_stuff_counter == sizeof(rpc::header)) {
        this->read_byte(&byte);
        this->read_stuff_counter = 0;
    }

    this->read_byte(&byte);
    if(byte == rpc::header[0]) {
        this->read_stuff_counter++;
    } else {
        this->read_stuff_counter = 0;
    }

    return byte;
}

void SerialPort::com_start_frame_transmission(rpc::Frame_Type frame_type) {
    auto t_byte = static_cast<uint8_t>(frame_type);

    this->write_stuff_counter = 0;
    this->write_byte(&stuffing_byte);
    this->write_word(rpc::header, sizeof(rpc::header));
    this->write_byte(&t_byte);
}

void SerialPort::close_port() {
    close(this->serial_port);
}

void SerialPort::set_exclusive_access() {
    if(flock(this->serial_port, LOCK_EX | LOCK_NB) == -1) {
        throw std::runtime_error("Serial port " + std::string(this->port_name) + " is already in use");
    }
}

void SerialPort::flush() {
    tcflush(this->serial_port, TCIFLUSH);
}

void SerialPort::set_default_config() {
    this->open_serial();
    
    struct termios serial_port_config;

    if(tcgetattr(this->serial_port, &serial_port_config) < 0) {
        printf("Error %i get config: %s\n", errno, std::strerror(errno));
        exit(errno);
    }

    serial_port_config.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL);
    serial_port_config.c_oflag &= ~(OPOST | ONLCR);
    serial_port_config.c_oflag |= (OFILL);
    serial_port_config.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    serial_port_config.c_cflag |= (CS8 | CREAD | CLOCAL);
    serial_port_config.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO |ECHOE | ECHOK | ECHOCTL | ECHOKE);
    serial_port_config.c_lflag |= (NOFLSH | TOSTOP);

    cfsetispeed(&serial_port_config, B115200);
    cfsetospeed(&serial_port_config, B115200);

    if(tcsetattr(this->serial_port, TCSANOW ,&serial_port_config) < 0) {
        printf("Error %i set config: %s\n", errno, std::strerror(errno));
        exit(errno);
    }

}