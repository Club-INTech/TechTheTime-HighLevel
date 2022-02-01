#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//T : type de service / message /// (Pas besoin pour sub :: Tr : T::Request / T::Msg? /// Rs... : arguments de msg)

template<class T>
class Subscriber : public rclcpp::Node
{
public:
    Subscriber()
        : Node("subscriber")
    {
        subscription_ = this->create_subscription<T>("topic", 10, std::bind(&Subscriber::topic_callback, this, _1)); //?????
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};