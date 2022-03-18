#include "client/ActionClient.hpp"
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <order_codes.hpp>
#include "subscriber/MotionSubscriber.hpp"
#include <thread>
#include "action_msg_srv/srv/order.hpp"
#include <stdexcept>
#include "robot_motion/RobotMotion.hpp"


using namespace std;

/*! \mainpage 
 *
 * \section introduction Introduction:
 * 
 * This is the high level code for the autonomic robot designed to participate in Erobot2022 robotics cup.
 * It is developed using ROS2(foxy) and has 5 nodes:
 * <ul>
 * <li>microcontroller_proxy</li>
 * <li>manager</li>
 * <li>robot_vision</li>
 * <li>motion_control</li>
 * <li>urg_node</li>
 * </ul>
 * 
 * Each node is represented by doxygen module. Each class contains a reference back to its module/node.
 * 
 * You have the brief description  and the reference to each node below.
 * 
 * \subsection intro1 microcontroller_proxy
 * 
 * microcontroller_proxy node is used to ineteract with microcontroller. It treats orders coming from other nodes
 * and then transforms them and sends them to the microcontroller. In the case of motion orders, it provides a feedback from the 
 * microcontroller.
 * 
 * Link to the @ref microcontroller_proxy
 * 
 * \subsection intro2 manager
 * 
 * manager node is responsible for the orders distribution and robot's position tracking. It sends data to the motion_control node and requests robot_vision and
 * microcontroller_proxy nodes to execute some orders. It takes the user action script as input.
 * 
 * Link to the @ref manager
 * 
 * \subsection intro3 robot_vision
 * 
 * robot_vision is responsible for image processing, distance measuring and aruco tag scanning. It receives requests from 
 * manager.
 * 
 * \subsection motion_control
 * 
 * motion_control is responsible for area scanning, path finding and sending basic orders to the microcontroller_proxy node.
 * 
 * Link to the @ref motion
 * 
 * \subsection urg_node
 * 
 * urg_node is a default ROS2 node, which publishes the points data of lidar. 
 * 
 * <br>
 * <hr>
 * <br>
 * 
 * You have a complete diagram of high level functionning below.
 * 
 * \section sheme The high level diagram:
 * 
 * 
 * 
 * \image html ROS2_Architecture.png
 * 
 * 
 * <br>
 * <br>
 * <hr>
 * <br>
 * 
 * \section sequence_diagram The example of functionning of the manager <-> microcontroller_proxy communication.
 * 
 * \image html ManagerMicrocontrollerFunctionning.png
 * 
 */

// Main page and manager.cpp documentation separator
// ======================================================================================================================


double RobotMotion::x = 0.0;
double RobotMotion::y = 0.0;
double RobotMotion::angle = 0.0;

/**
 * 
 * @ingroup manager
 * 
 * manager.cpp is an entry point of the manager node.
 * 
 * @details We initialize ros2 client library <a target="blank" href="https://docs.ros2.org/foxy/api/rclcpp/index.html">rclcpp</a>
 * Then we create a shared pointers to clients which inherit from @ref ClientT. We call @ref ClientT#set_shared to set a self shared pointer which is needed 
 * for ros2 client methods. Finally we can use @ref ClientT#wait_for_connection method to connect to the service. We can then use
 * @ref ClientT#send method.
 * 
 * @author sudogauss
*/

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto commClient = std::make_shared<ActionClient>();
    commClient->set_shared(commClient);
    commClient->wait_for_connection(); 

    std::thread subscriber_thread([](){
        rclcpp::spin(std::make_shared<MotionSubscriber>());
    });

    std::thread client_thread([&commClient](){
        try {
            auto res = commClient->send((int64_t) OrderCodes::MOVE, 1000, 0, 0);

            MotionStatusCodes status = static_cast<MotionStatusCodes>(res.get()->motion_status);
            if(status == MotionStatusCodes::COMPLETE) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: finished");
            } else if(status == MotionStatusCodes::NOT_COMPLETE){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion was not complete");
            } else if(status == MotionStatusCodes::MOTION_TIMEOUT) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion has been timed out");
            }

            res = commClient->send((int64_t) OrderCodes::MOVE, 500, 0, 0);
        } catch(const std::runtime_error& e) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", e.what());
        }  
    });

    subscriber_thread.join();
    client_thread.join();

    rclcpp::shutdown();

    return 0;
}