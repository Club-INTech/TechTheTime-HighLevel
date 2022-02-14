#include "rclcpp/rclcpp.hpp"
#include "example_topic/msg/example.hpp"
#include "subscriberClass.hpp"
#include <iostream>

class TestSubscriber : public Subscriber<std::string> {
public:
    TestSubscriber() : Subscriber("test") {};

private:
    void topic_callback(const std::string::Shared_ptr msg) {
        std::cout << "Message recieved" << msg << std::endl;
    }
};