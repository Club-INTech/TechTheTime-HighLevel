#include "SerialPort.hpp"
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

    unsigned int control_mode_bits[] = {PARENB, CSTOPB, CRTSCTS, CLOCAL, CREAD};
    unsigned int local_modes_bits[] = {ICANON, ECHO, ECHOE, ECHONL, ISIG};
    unsigned int input_modes_bits[] = {IXON, IXOFF, IXANY, IGNBRK, BRKINT, PARMRK, ISTRIP, INLCR, IGNCR, ICRNL};
    unsigned int output_modes_bits[] = {OPOST, ONLCR};

    uint8_t stuffing_byte = ~rpc::header[0];
    // k2o::dispatcher dispatcher{rpc::master::keyring};
}

SerialPort::SerialPort(const char* port_name) {
    this->port_name = port_name;
    this->write_stuff_counter = 0;
    this->read_stuff_counter = 0;
    this->new_response = false;
}

void SerialPort::open_serial() {
    int serial_port  = open(this->port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    sleep(1);
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
            this->serial_port_config.c_cflag |= control_mode_bits[std::distance(control_bits_state.begin(), it)];
        } else {
            this->serial_port_config.c_cflag &= ~control_mode_bits[std::distance(control_bits_state.begin(), it)];
        }    
    }
}

void SerialPort::configure_local_modes(std::initializer_list<bool> local_bits_state) {

    assert(local_bits_state.size() == 5);    

    for(auto it = local_bits_state.begin(); it < local_bits_state.end(); it++) {
        if(*it) {
            this->serial_port_config.c_lflag |= control_mode_bits[std::distance(local_bits_state.begin(), it)];
        } else {
            this->serial_port_config.c_lflag &= ~control_mode_bits[std::distance(local_bits_state.begin(), it)];
        }    
    }
}

void SerialPort::configure_input_modes(std::initializer_list<bool> input_bits_state) {

    assert(input_bits_state.size() == 10);    

    for(auto it = input_bits_state.begin(); it < input_bits_state.end(); it++) {
        if(*it) {
            this->serial_port_config.c_iflag |= control_mode_bits[std::distance(input_bits_state.begin(), it)];
        } else {
            this->serial_port_config.c_iflag &= ~control_mode_bits[std::distance(input_bits_state.begin(), it)];
        }    
    }
}

void SerialPort::configure_output_modes(std::initializer_list<bool> output_bits_state) {

    assert(output_bits_state.size() == 2);    

    for(auto it = output_bits_state.begin(); it < output_bits_state.end(); it++) {
        if(*it) {
            this->serial_port_config.c_oflag |= control_mode_bits[std::distance(output_bits_state.begin(), it)];
        } else {
            this->serial_port_config.c_oflag &= ~control_mode_bits[std::distance(output_bits_state.begin(), it)];
        }    
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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Blocks");
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
    sleep(1);
    tcflush(this->serial_port, TCIFLUSH);
}

void SerialPort::set_default_config() {
    this->open_serial();
    this->get_config();

    // fcntl(this->serial_port, F_SETFL, 0);

    this->configure_control_modes(CS8, {
        false,
        false,
        false,
        true,
        true
    });

    this->configure_local_modes({
        true,
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

    // this->define_blocking_mode(scom::BlockingModes::NO_BLOCK, {});
    this->set_input_speed(115200);
    this->set_output_speed(115200);

    // this->flush();

    this->set_config();
}