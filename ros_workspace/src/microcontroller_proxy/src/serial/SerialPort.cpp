#include "SerialPort.hpp"
#include <cerrno>
#include <cstring>
#include <vector>
#include <cassert>
#include <unistd.h>
#include <sys/file.h>
#include <stdexcept>

using namespace scom;

namespace scom {

    unsigned int control_mode_bytes[] = {PARENB, CSTOPB, CRTSCTS, CLOCAL, CREAD};
    unsigned int local_modes_bytes[] = {ICANON, ECHO, ECHOE, ECHONL, ISIG};
    unsigned int input_modes_bytes[] = {IXON, IXOFF, IXANY, IGNBRK, BRKINT, PARMRK, ISTRIP, INLCR, IGNCR, ICRNL};
    unsigned int output_modes_bytes[] = {OPOST, ONLCR};
}

SerialPort::SerialPort(const char* port_name) {
    this->port_name = port_name;
}

void SerialPort::open_serial() {
    int serial_port  = open(this->port_name, O_RDWR);
    if (serial_port < 0) {
        printf("Error %i open: %s\n", errno, std::strerror(errno));
        exit(errno);
    }
    this->serial_port = serial_port;
}

void SerialPort::set_config() {
    if(tcsetattr(this->serial_port, TCSANOW, &this->serial_port_config) != 0) {
        printf("Error %i set config: %s\n", errno, std::strerror(errno));
        exit(errno);
    }
}

void SerialPort::get_config() {
    if(tcgetattr(this->serial_port, &this->serial_port_config) != 0) {
        printf("Error %i get config: %s\n", errno, std::strerror(errno));
        exit(errno);
    }
}

void SerialPort::set_input_speed(speed_t baud_rate) {
    cfsetispeed(&this->serial_port_config, baud_rate);
}

void SerialPort::set_output_speed(speed_t baud_rate) {
    cfsetospeed(&this->serial_port_config, baud_rate);
}

void SerialPort::define_blocking_mode(BlockingModes mode, std::initializer_list<int> args) {
    if(mode == BlockingModes::NO_BLOCK) {
        this->serial_port_config.c_cc[VTIME] = 0;
        this->serial_port_config.c_cc[VMIN] = 0;
    } else if (mode == BlockingModes::FIXED_BYTES) {
        assert(args.size() == 1);
        this->serial_port_config.c_cc[VTIME] = 0;
        auto it = args.begin();
        this->serial_port_config.c_cc[VMIN] = *it;
    } else if (mode == BlockingModes::TIMEOUT) {
        assert(args.size() == 1);
        this->serial_port_config.c_cc[VMIN] = 0;
        auto it = args.begin();
        this->serial_port_config.c_cc[VTIME] = *it;
    } else if (mode == BlockingModes::TIMEOUT_WITH_FIXED_BYTES) {
        assert(args.size() == 2);
        auto it = args.begin();
        this->serial_port_config.c_cc[VTIME] = *it;
        it++;
        this->serial_port_config.c_cc[VMIN] = *it;
    }
    

}

void SerialPort::configure_control_modes(tcflag_t bits_per_byte, std::initializer_list<bool> control_bits_state) {
    this->serial_port_config.c_cflag &= ~CSIZE;
    this->serial_port_config.c_cflag |= bits_per_byte;

    assert(control_bits_state.size() == 5);    

    for(auto it = control_bits_state.begin(); it < control_bits_state.end(); it++) {
        if(*it) {
            this->serial_port_config.c_cflag |= control_mode_bytes[std::distance(control_bits_state.begin(), it)];
        } else {
            this->serial_port_config.c_cflag &= ~control_mode_bytes[std::distance(control_bits_state.begin(), it)];
        }    
    }
}

void SerialPort::configure_local_modes(std::initializer_list<bool> local_bits_state) {

    assert(local_bits_state.size() == 5);    

    for(auto it = local_bits_state.begin(); it < local_bits_state.end(); it++) {
        if(*it) {
            this->serial_port_config.c_lflag |= control_mode_bytes[std::distance(local_bits_state.begin(), it)];
        } else {
            this->serial_port_config.c_lflag &= ~control_mode_bytes[std::distance(local_bits_state.begin(), it)];
        }    
    }
}

void SerialPort::configure_input_modes(std::initializer_list<bool> input_bits_state) {

    assert(input_bits_state.size() == 10);    

    for(auto it = input_bits_state.begin(); it < input_bits_state.end(); it++) {
        if(*it) {
            this->serial_port_config.c_iflag |= control_mode_bytes[std::distance(input_bits_state.begin(), it)];
        } else {
            this->serial_port_config.c_iflag &= ~control_mode_bytes[std::distance(input_bits_state.begin(), it)];
        }    
    }
}

void SerialPort::configure_output_modes(std::initializer_list<bool> output_bits_state) {

    assert(output_bits_state.size() == 2);    

    for(auto it = output_bits_state.begin(); it < output_bits_state.end(); it++) {
        if(*it) {
            this->serial_port_config.c_oflag |= control_mode_bytes[std::distance(output_bits_state.begin(), it)];
        } else {
            this->serial_port_config.c_oflag &= ~control_mode_bytes[std::distance(output_bits_state.begin(), it)];
        }    
    }
}

ssize_t SerialPort::write_byte(uint8_t byte) {
    return write(this->serial_port, &byte, 1);
}

ssize_t SerialPort::read_byte(uint8_t* byte) {
    return read(this->serial_port, byte, 1);
}

ssize_t SerialPort::read_word(uint8_t* word, int size) {
    return read(this->serial_port, word, size);
}

void SerialPort::close_port() {
    close(this->serial_port);
}

void SerialPort::set_exclusive_access() {
    if(flock(this->serial_port, LOCK_EX | LOCK_NB) == -1) {
        throw std::runtime_error("Serial port " + std::string(this->port_name) + " is already in use");
    }
}

void SerialPort::set_default_config() {
    this->open_serial();
    this->get_config();

    this->configure_control_modes(CS8, {
        false,
        false,
        false,
        true,
        true
    });

    this->configure_local_modes({
        false,
        false,
        false,
        false,
        false
    });

    this->configure_input_modes({
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false
    });

    this->configure_output_modes({
        false,
        false
    });

    this->define_blocking_mode(scom::BlockingModes::TIMEOUT_WITH_FIXED_BYTES, {1000, 6});
    this->set_input_speed(115200);
    this->set_output_speed(115200);

    this->set_config();
}