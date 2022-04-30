#! /bin/bash

path=$PWD
path_to_order_srv=$path/../ros_workspace/src/srvs_msgs/action_msg_srv/srv/Order.srv
path_to_order_header=$path/../ros_workspace/src/srvs_msgs/install/action_msg_srv/include/action_msg_srv/srv/detail/order__struct.hpp

order_regex="Order_Request_"

reset="$(tput sgr0)"
red="$(tput bold ; tput setaf 1)"
green="$(tput bold ; tput setaf 2)"
yellow="$(tput bold ; tput setaf 3)"

available_nodes=(manager microcontroller_proxy motion_control urg_node)
srvs_msgs=(action_msg_srv motion_msg_srv)

check_node_exists () {
    for available_node in ${available_nodes[@]}; do
        if [[ "$available_node" == "$1" ]]; then
            return 0
        fi
    done

    return 1
}

fail_on_build () {
    echo -e "${red}Failed to build $1 ${reset}"
    exit 1
}

print_help () {
    echo ""
    echo "Usage: ./build-hl.sh [options] [-p \"<nodes_name>\"]"
    echo ""
    echo "options are:"
    echo ""
    echo "-h for help"
    echo "-r for rebuild without cache"
    echo "-v for verbose. If you use -v all backlog of colcon will be showed"
    echo ""

    echo "If you use -p only given nodes will be built"
    echo "Available nodes are:"
    echo ${available_nodes[@]}

    echo ""
    echo "${green}Good luck!${reset}"
    echo ""
}

p_flag='false' # package flag
v_flag='false' # verbose flag
h_flag='false' # help flag
r_flag='false' #rebuild flag
nodes=(${available_nodes[@]})

while getopts 'hrvp:' flag; do
  case "${flag}" in
    h) h_flag='true' ;;
    v) v_flag='true' ;;
    r) r_flag='true' ;;
    p) nodes=(${OPTARG})
        p_flag='true' ;;
    *) print_help
       exit 1 ;;
  esac
done

if [ $h_flag = 'true' ]; then
    print_help
    exit 0
fi

if [ $p_flag = 'true' ]; then
    for node in "${nodes[@]}"; do 
        check_node_exists $node
        if [ $? -ne 0 ]; then
            echo "Node $node does not exist"
            print_help
            exit 1
        fi
    done
fi

if [ $r_flag = 'true' ]; then
    cd ../ros_workspace
    rm -rf build
    rm -rf install
    rm -rf log
    cd src/srvs_msgs
    rm -rf build
    rm -rf install
    rm -rf log
    cd ../../../build_tool
fi

ostream=/dev/null

if [ $v_flag = 'true' ]; then
    ostream=1
fi

echo "${yellow}===== Building interfaces =====${reset}"

echo "${yellow}===== Building action_msg_srv =====${reset}"

cd ../ros_workspace/src/srvs_msgs

for srv_msg in "${srvs_msgs[@]}"; do
    echo "${yellow}===== Building $srv_msg =====${reset}"
    colcon build --packages-select $srv_msg >&$ostream
    if [ $? -ne 0 ]; then
        fail_on_build "action_msg_srv"
    fi
    echo "${green}===== Finished $srv_msg =====${reset}"
done

source install/setup.bash
cd $path

echo "${green}===== Finished action_msg_srv =====${reset}"

echo "${yellow}===== Generating request constructor =====${reset}"

python3 request_constructor_generator.py $path_to_order_header $path_to_order_srv $order_regex

echo "${green}===== Finished constructor generator =====${reset}"

echo "${green}===== Finished interfaces build =====${reset}"

echo "${yellow}===== Building nodes =====${reset}"

cd ../ros_workspace

for node in "${nodes[@]}"; do
    echo "${yellow}===== Building $node =====${reset}"

    colcon build --packages-select $node >&$ostream
    if [ $? -ne 0 ]; then
        fail_on_build $node
    fi

    echo "${green}===== Finished $node =====${reset}"
done

echo "${green}===== Finished nodes=====${reset}"

echo "${green}===== Finished build =====${reset}"

exit 0