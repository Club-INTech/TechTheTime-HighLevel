#! /bin/bash

echo "Starting interceptty"

sudo interceptty -o /dev/null -s "ispeed 115200 ospeed 115200" /dev/ttyACM0 /dummy &

echo "Run microcontroller_proxy"

cd ../ros_workspace

source install/setup.bash

cd src/microcontroller_proxy/src

ros2 run microcontroller_proxy microcontroller_proxy $PWD/config.yaml &

ros2 run manager manager &