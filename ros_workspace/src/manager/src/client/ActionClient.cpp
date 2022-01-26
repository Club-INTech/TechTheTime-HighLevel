#include "rclcpp/rclcpp.hpp"
#include "action_srv/srv/order.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "ActionClient.hpp"
#include "struct_wrapper.hpp"

template<typename T, typename... Rs>
ActionClientNode::ActionClientNode(const std::string& client_name) : 
  node(rclcpp::Node::make_shared<rclcpp::Node>(client_name)),
  request(std::make_shared<T::Request>())
{
  client = node->create_client<T>(client_name);
}

template<typename T, typename... Rs>
bool ActionClientNode::wait_for_connection() {
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  return true;
}

template<typename T, typename... Rs>
auto send(Rs... args) {
  request->set_values(args...);
  auto result = client->async_send_request(request::value);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->success);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }
}