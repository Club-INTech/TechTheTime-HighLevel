#! /bin/bash

trap "kill 0" SIGINT
# trap 'kill $(jobs -p | xargs)' SIGINT SIGHUP SIGTERM EXIT

echo "Starting interceptty"

sudo interceptty -o /dev/null -s "ispeed 115200 ospeed 115200" /dev/ttyACM0 /dummy &

sleep 2

echo "Run microcontroller_proxy"

cd ../ros_workspace

source install/setup.bash

# (ros2 run urg_node urg_node_driver --ros-args --params-file ./install/urg_node/share/urg_node/launch/urg_node_ethernet.yaml) &

# sleep 2

# (ros2 run motion_control motion_control) &

# sleep 2

(cd src/microcontroller_proxy/src && ros2 run microcontroller_proxy microcontroller_proxy $PWD/config.yaml) &

sleep 5

(cd src/manager/src && ros2 run manager manager $PWD/config.yaml) &

sleep 600
