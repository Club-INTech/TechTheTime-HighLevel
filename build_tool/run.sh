#! /bin/bash

trap "kill 0" SIGINT

config_folder=$PWD/config
script_folder=$PWD/script

h_flag='false'
v_flag='false' # verbose flag

mode='full'
script='qualification'

print_help () {
    echo ""
    echo "Usage: ./run.sh [options] [-m [full|basic]] [-s <script_name>]"
    echo ""
    echo "options are:"
    echo ""
    echo "-h for help"
    echo "-v for verbose. If you use -v all logs of all nodes will be showed"
    echo ""

    echo "You can provide a mode (basic or full) with -m. In basic mode there is only script execution."
    echo "In full mode there is script and path-finding. It is useful for matches of the robotics cup."

    echo "Use -s to provide a script that will be executed"

    echo ""
    echo "${green}Good luck!${reset}"
    echo ""
}

while getopts 'hvm:s:' flag; do
  case "${flag}" in
    h) h_flag='true';;
    v) v_flag='true' ;;
    m) mode=${OPTARG} ;;
    s) script=${OPTARG} ;
    *) echo "The only available flag is verbose [-v]"
       exit 1 ;;
  esac
done

ostream=/dev/null

if [ $h_flag = 'true' ]; then
    print_help
    exit 0
fi

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

sudo interceptty -o $ostream -s "ispeed 115200 ospeed 115200" /dev/ttyACM0 /dummy &

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
