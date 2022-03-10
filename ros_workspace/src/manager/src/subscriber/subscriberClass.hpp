#include "rclcpp/rclcpp.hpp"

//T : type de service / message /// (Pas besoin pour sub :: Tr : T::Request / T::Msg? /// Rs... : arguments de msg)

template<class T>
class Subscriber : public rclcpp::Node
{
    using shared_ptr_T = typename T::SharedPtr;
    using Subscriber_T = typename rclcpp::Subscription<T>::SharedPtr;

public:
    Subscriber(std::string name)
        : Node("subscriber")
    {
        subscription_ = this->create_subscription<T>(name, 10, std::bind(&Subscriber::topic_callback, std::placeholders::_1));
    }

private:
    virtual void topic_callback(const shared_ptr_T msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.get());
    }
    Subscriber_T subscription_;
};