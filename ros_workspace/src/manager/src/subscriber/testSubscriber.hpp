#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "subscriberClass.hpp"
#include <iostream>

class TestSubscriber : public Subscriber<std::string> {
public:
    TestSubscriber() : Subscriber("test") {};

private:
    void topic_callback(const std::string::Shared_ptr msg) {
        std::cout << msg << std::endl;
    }
};