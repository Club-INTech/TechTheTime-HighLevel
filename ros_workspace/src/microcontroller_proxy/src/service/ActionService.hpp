#ifndef ACTION_SERVER_HPP
#define ACTION_SERVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "action_msg_srv/srv/order.hpp"
#include "order_binder.hpp"

#include <memory>
#include <string>

#include "../serial/SerialPort.hpp"
#include "../publisher/MotionPublisher.hpp"

/** @defgroup microcontroller_proxy Microcontroller node.
 * @{
*/ 

/**
 * ActionService is a <a target="_blank" href="https://docs.ros.org/en/foxy/Tutorials/Services/Understanding-ROS2-Services.html">ROS2 service node</a>, 
 * which accepts a request(defined by action_msg_srv) from ClientT and executes
 * a corresponding function.
 * 
 * It can be considered as a node that wraps a ROS2 service and pre-treat incoming requests from manager or motion control.
 * That's why it is a part of {@link ../microcontroller_proxy.cpp}
 * 
 * It inherits from <a target="_blank" href="https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Node.html">rclcpp::Node</a>
 * 
 * @author sudogauss
*/ 
class ActionService : public rclcpp::Node {

    
public:

    /**
     * An alias to the request type defined in action_msg_srv 
    */ 
    using request_T = typename action_msg_srv::srv::Order::Request;

    /**
     * An alias to the response type defined in action_msg_srv
    */
    using response_T = typename action_msg_srv::srv::Order::Response;

    /**
     * An alias to the shared pointer of request type defined in action_msg_srv
    */
    using shared_request_T = typename std::shared_ptr<action_msg_srv::srv::Order::Request>;

    /**
     * An alias to the shared pointer of response type defined in action_msg_srv
    */
    using shared_response_T = typename std::shared_ptr<action_msg_srv::srv::Order::Response>;


    /**
     * ActionService constructor
     * 
     * @param service_name is a name of ROS2 service, which serves for identifier(it must be unique)
     * 
     * ActionService::ActionService calls a super constructor of rclcpp::Node with a service_name parameter,
     * initializes ActionService::service_name, creates new ActionService::order_binder and new ActionService::service.
     * 
     * It uses a <b>create_service</b> method of ActionService inherited from <b>rclcpp::Node</b>, which allows to create
     * a ROS2 service that uses a particular service message interface (<b>action_msg_srv::srv::Order</b> in this case,
     * see <i>ros_workspace/src/srvs_msgs/action_msg_srv/srv/Order.srv for more information</i>). This method take a
     * service_name and a function to be executed on request received. As a function to treat requests we use
     * ActionService::treat_orders.
     * 
     * Then we initialize ActionService::microcontroller_gateway (is used to send data to microcontroller over serial).
     * 
     * Finally, with the help of ActionService::order_binder we bind a function to execute on order code recieved from 
     * request. Basically you can use the following code snippet to bind to an order:
     * 
     * order_binder.bind_order(<your_order_code_here>, [&](shared_request_T req, shared_response_T res) {
     *      ...
     *      <treat req and send res there>
     *      ...
     * }); 
     * 
     * All order codes are defined there order_codes.hpp.
    */ 
    ActionService(const std::string& service_name);

    /**
     * 
    */ 
    void treat_orders(const shared_request_T req, shared_response_T res);
    
    /**
     * ActionService::microcontroller_gateway is a unique pointer to the SerialPort.
     * 
     * It is used to send data to the microcontroller. You can fully customize serial communication(over UART).
     * You can send data with scom::SerialPort::write_byte or scom::SerialPort::write_word 
     * and read data with scom::SerialPort::read_byte or scom::SerialPort::read_word.
     * 
     * You can also use scom::SerialPort::call_remote_function to call an order on the lowlevel side(microcontroller).
    */ 
    std::shared_ptr<scom::SerialPort> microcontroller_gateway;

private:

    /**
     * order_binder is used to redirect received requests by calling a function binded to an order code.
     * Take a look at OrderBinder documentation to better understand it functionning.
     *  
    */ 
    OrderBinder<shared_request_T, shared_response_T> order_binder;

    /**
     * name of the service 
    */ 
    std::string service_name;

    /**
     * A shared pointer to ROS2 service, which is responsable to receive requests and send responses 
    */ 
    rclcpp::Service<action_msg_srv::srv::Order>::SharedPtr service;
    std::shared_ptr<MotionPublisher> motion_publisher;
    
};

/** @} */

#endif