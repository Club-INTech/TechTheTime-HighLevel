#include "client/ActionClient.hpp"
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <action_msg_srv_shared/order_codes.hpp>
#include "subscriber/MotionSubscriber.hpp"
#include <thread>
#include "action_msg_srv/srv/order.hpp"
#include <stdexcept>
#include "robot_status/RobotStatus.hpp"
#include "dev/order_reader.hpp"
#include <tuple>
#include <csignal>
#include "yaml-cpp/yaml.h"
#include "script/script.hpp"

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
 * Link to the motion_control.py
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


double RobotStatus::x = 0;
double RobotStatus::y = 0;
double RobotStatus::angle_ = 0;
double RobotStatus::angle = 0;
Team RobotStatus::team = Team::NONE;
Robot RobotStatus::robot = Robot::NONE;

void terminate(int code) {
    rclcpp::shutdown();
    exit(code);
}

template<typename T>
T process_element(YAML::Node* config, const char* elem) {
    if(!(*config)[elem]) {
        std::cout << "Parameter " << elem << " is required" << std::endl;
        terminate(1);
    } else {
        return (*config)[elem].as<T>();
    } 
}

void init(YAML::Node* config) {
    RobotStatus::team = (process_element<std::string>(config, "team").compare("yellow")) ? Team::PURPLE : Team::YELLOW;
    RobotStatus::robot = (process_element<std::string>(config, "robot").compare("master")) ? Robot::SLAVE : Robot::MASTER;

    if(RobotStatus::team == Team::YELLOW) {
        
        if(RobotStatus::robot == Robot::MASTER) {

            RobotStatus::x = START_X_1A_YELLOW;
            RobotStatus::y = START_Y_1A_YELLOW;
            RobotStatus::angle_ = START_ANGLE_1A_YELLOW;
            RobotStatus::angle = START_ANGLE_1A_YELLOW;

        } else if(RobotStatus::robot == Robot::SLAVE) {

            RobotStatus::x = START_X_2A_YELLOW;
            RobotStatus::y = START_Y_2A_YELLOW;
            RobotStatus::angle_ = START_ANGLE_2A_YELLOW;
            RobotStatus::angle = START_ANGLE_2A_YELLOW;

        }

    } else if(RobotStatus::team == Team::PURPLE) {

        if(RobotStatus::robot == Robot::MASTER) {

            RobotStatus::x = START_X_1A_PURPLE;
            RobotStatus::y = START_Y_1A_PURPLE;
            RobotStatus::angle_ = START_ANGLE_1A_PURPLE;
            RobotStatus::angle = START_ANGLE_1A_PURPLE;

        } else if(RobotStatus::robot == Robot::SLAVE) {
            
            RobotStatus::x = START_X_2A_PURPLE;
            RobotStatus::y = START_Y_2A_PURPLE;
            RobotStatus::angle_ = START_ANGLE_2A_PURPLE;
            RobotStatus::angle = START_ANGLE_2A_PURPLE;

        }
    }
}

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

    std::signal(SIGINT, terminate);

    rclcpp::init(argc, argv);

    if(argc < 2) {
        std::cout << "Please provide a full path to your config file. Example: $PWD/config.yaml" << std::endl;
        terminate(1);
    }

    std::string filename(argv[1]);
    YAML::Node config = YAML::LoadFile(filename);

    std::thread subscriber_thread([](){
        rclcpp::spin(std::make_shared<MotionSubscriber>());
    });

    std::thread client_thread([](){

            Script script = Script();
            script.pushOrder(std::bind(&Script::angleABS, script, M_PI/4,0,0));

            script.wait_for_jumper();
            script.run();
    });


    subscriber_thread.join();
    client_thread.join();

    rclcpp::shutdown();

    return 0;
}