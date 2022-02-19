#include "rclcpp/rclcpp.hpp"
#include "example_topic/msg/example.hpp"
#include "subscriberClass.hpp"
#include <iostream>

class TestSubscriber : public Subscriber<example_topic::msg::Example> {
public:
    TestSubscriber() : Subscriber("test") {};

private:
    void topic_callback(const example_topic::msg::Example::SharedPtr msg) {
        std::cout << "Message recieved" << msg << std::endl;
    }
};