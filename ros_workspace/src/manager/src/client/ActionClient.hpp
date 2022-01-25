#ifndef ACTION_CLIENT_HPP
#define ACTION_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "action_msg_srv/srv/order.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include "struct_wrapper.hpp"

template<typename T, typename... Rs>
class ActionClientNode : public rclcpp::Node {
public:
  ActionClientNode() = delete;
  ActionClientNode(const std::string&);
  bool wait_for_connection();
  auto send(Rs...);

private:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Client<T>::SharedPtr client;
  struct_wrapper<T, Rs...> request;

};

#endif