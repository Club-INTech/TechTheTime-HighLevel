#ifndef ACTION_CLIENT_HPP
#define ACTION_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "action_msg_srv/srv/order.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include "struct_wrapper.hpp"

using namespace std::chrono_literals;

template<typename T, typename Tr, typename... Rs>
class ActionClientNode : public rclcpp::Node {

  using shared_ptr_T = typename rclcpp::Client<T>::SharedPtr;

public:
  ActionClientNode(const std::string& client_name) : Node(client_name), client(this->create_client<T>(client_name)) {
  }

  bool wait_for_connection() {
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    return true;
  }

  auto send(Rs... args) {
    request->set_values(args...);
    auto result = client->async_send_request(request->value);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->success);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
    }
  }

private:
  // std::shared_ptr<rclcpp::Node> node;
  shared_ptr_T client;
  struct_wrapper<Tr, Rs...> request;

};

#endif