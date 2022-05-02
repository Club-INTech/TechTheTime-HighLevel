#! /bin/bash

trap "kill 0" SIGINT
# trap 'kill $(jobs -p | xargs)' SIGINT SIGHUP SIGTERM EXIT

echo "Starting interceptty"

sudo interceptty -o /dev/null -s "ispeed 115200 ospeed 115200" /dev/ttyACM0 /dummy &

sleep 2

echo "Starting microcontroller_proxy"

cd ../ros_workspace

source install/setup.bash

# (ros2 run urg_node urg_node_driver --ros-args --params-file ./install/urg_node/share/urg_node/launch/urg_node_ethernet.yaml) &

# sleep 2

# (ros2 run motion_control motion_control) &

# sleep 2

(cd src/microcontroller_proxy/src && ros2 run microcontroller_proxy microcontroller_proxy $PWD/config.yaml &>/dev/null) &

sleep 5

echo "Starting manager"

(cd src/manager/src && ros2 run manager manager $PWD/config.yaml &>/dev/null $PWD/user_script/$1.bot) &

sleep 600
