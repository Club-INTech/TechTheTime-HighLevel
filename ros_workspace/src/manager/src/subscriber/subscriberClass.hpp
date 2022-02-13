#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//T : type de service / message /// (Pas besoin pour sub :: Tr : T::Request / T::Msg? /// Rs... : arguments de msg)

template<class T>
class Subscriber : public rclcpp::Node
{
public:
    Subscriber(std::string name)
        : Node("subscriber")
    {
        subscription_ = this->create_subscription<T>(name, 10, std::bind(&Subscriber::topic_callback, this, _1));
    }

private:
    virtual void topic_callback(const T::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.get());
    }
    rclcpp::Subscription<T>::SharedPtr subscription_;
};