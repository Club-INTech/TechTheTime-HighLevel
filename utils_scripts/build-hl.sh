#! /bin/bash

path=$PWD
path_to_order_srv=$path/../ros_workspace/src/srvs_msgs/action_msg_srv/srv/Order.srv
path_to_order_header=$path/../ros_workspace/src/srvs_msgs/install/action_msg_srv/include/action_msg_srv/srv/detail/order__struct.hpp
order_regex="Order_Request_"

echo "===== Building interfaces ====="

cd ../ros_workspace/src/srvs_msgs
colcon build --packages-select action_msg_srv
source install/setup.bash
cd $path

echo "===== Finished ====="

echo "===== Generating request constructor ====="

python3 request_constructor_generator.py $path_to_order_header $path_to_order_srv $order_regex

echo "===== Finished ====="

echo "===== Building nodes ====="

cd ../..
colcon build --packages-select manager
source install/setup.bash

echo "===== Finished ====="


exit 0