#ifndef CLIENT_CLASS_HPP
#define CLIENT_CLASS_HPP

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include "struct_wrapper.hpp"
#include <stdexcept>

using namespace std::chrono_literals;

/** @defgroup manager Manager node.
 * @{
*/

/**
 * ClientT class. 
*/ 
template<class T, class Treq, class... Rs>
class ClientT : public rclcpp::Node {

public:

  using shared_ptr_T = typename rclcpp::Client<T>::SharedPtr;
  using shared_future_T = typename rclcpp::Client<T>::SharedFuture;

  ClientT(const std::string& client_name) : Node(client_name) {
    this->client_name = client_name;
    this->client = this->create_client<T>(client_name);
  }

  void set_shared(std::shared_ptr<ClientT<T, Treq, Rs...>> ptr) {
    this->self_ptr = ptr;
  }

  void wait_for_connection() {
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting...");
        exit(1);
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), this->client_name + " client is waiting for service...");
    }
  }

  auto send(Rs... args) {
    request.set_values(args...);
    auto result = client->async_send_request(request.value);
    if (rclcpp::spin_until_future_complete(this->self_ptr, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return result;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
      throw std::runtime_error("Failed to call service");
    }
  }

private:
  std::shared_ptr<ClientT<T, Treq, Rs...>> self_ptr;
  std::string client_name;
  shared_ptr_T client;
  struct_wrapper<Treq, Rs...> request;
};

/** @} */

#endif