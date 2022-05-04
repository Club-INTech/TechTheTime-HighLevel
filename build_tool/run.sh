#! /bin/bash

trap "kill 0" SIGINT

if [ $# -ne 2 ]; then
    echo "You must provide 2 arguments: mode and script."
    echo "Possible values:"
    echo "mode=(basic full)"
    echo "script=<name_of_your_script_in_script_folder>"
    exit 1
fi

mode=$1
config_folder=$PWD/config
script_folder=$PWD/script
script=$2

v_flag='false' # verbose flag

while getopts 'v' flag; do
  case "${flag}" in
    v) v_flag='true' ;;
    *) echo "The only available flag is verbose [-v]"
       exit 1 ;;
  esac
done

ostream=/dev/null

if [ $v_flag = 'true' ]; then
    ostream=1
fi

if [[ "$mode" != "basic" ]] && [[ "$mode" != "full" ]]; then
    echo "Mode $mode does not exist"
    echo "Authorized modes are basic and full"
    exit 1
fi

if [[ ! -f "$script_folder/$script.bot" ]]; then
    echo "Script $script does not exist"
    exit 1
fi

echo "Starting interceptty"

sudo interceptty -o /dev/null -s "ispeed 115200 ospeed 115200" /dev/ttyACM0 /dummy &

sleep 2

cd ../ros_workspace

echo "Starting microcontroller_proxy"


source install/setup.bash

(cd src/microcontroller_proxy/src && ros2 run microcontroller_proxy microcontroller_proxy $config_folder/microcontroller_proxy.yaml >&$ostream) &

sleep 3

if [[ "$mode" == "full" ]]; then

    echo "Starting urg_node"
    (ros2 run urg_node urg_node_driver --ros-args --params-file ./install/urg_node/share/urg_node/launch/urg_node_ethernet.yaml) &
    sleep 3

    echo "Starting motion_control"
    (ros2 run motion_control motion_control) &
    sleep 3
fi

echo "Starting manager"

(cd src/manager/src && ros2 run manager manager $config_folder/manager.yaml $script_folder/$script.bot >&$ostream) &

sleep 600
