#! /bin/bash

path=$PWD
path_to_order_srv=$path/../ros_workspace/src/srvs_msgs/action_msg_srv/srv/Order.srv
path_to_order_header=$path/../ros_workspace/src/srvs_msgs/action_msg_srv/install/action_msg_srv/include/action_msg_srv/srv/detail/order__struct.hpp
path_to_generator=$path/request_constructor_generator.py

path_to_manager=$path/../ros_workspace/src/manager
path_to_action_msg_srv=$path/../ros_workspace/src/srvs_msgs/action_msg_srv
order_regex="Order_Request_"

echo "===== Building interfaces ====="

cd $path_to_action_msg_srv
colcon build --packages-select action_msg_srv
source install/setup.bash
cd $path

echo "===== Finished ====="

echo "===== Generating request constructor ====="

python3 $path_to_generator $path_to_order_header $path_to_order_srv $order_regex

echo "===== Finished ====="

echo "===== Building nodes ====="

cd $path_to_manager
colcon build --packages-select manager
source $path_to_manager/install/setup.bash

echo "===== Finished ====="


exit 0