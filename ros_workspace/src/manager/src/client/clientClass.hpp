#ifndef CLIENT_CLASS_HPP
#define CLIENT_CLASS_HPP


#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include "struct_wrapper.hpp"
#include <stdexcept>


//using namespace std::chrono_literals;


template<class T, class Tr, class... Rs>
class ClientNode : public rclcpp::Node {

  using shared_ptr_T = typename rclcpp::Client<T>::SharedPtr;

public:
  ClientNode(const std::string& client_name) : Node(client_name), client(this->create_client<T>(client_name)), client_name(client_name) {
  }

  void wait_for_connection() {
    while (!client->wait_for_service(1)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        exit(1);
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), this->client_name + " is waiting for service...");
    }
  }

  auto send(Rs... args) {
    request->set_values(args...);
    auto result = client->async_send_request(request->value);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: %d", result.get()->success);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
    }
  }

private:
  std::string client_name;
  shared_ptr_T client;
  struct_wrapper<Tr, Rs...> request;

};

#endif