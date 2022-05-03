#!/bin/bash

green="$(tput bold ; tput setaf 2)"
reset="$(tput sgr0)"

cd ..

git submodule deinit --force --all

echo "===== Initializing submodules ====="

git submodule init
git submodule update --remote

echo "===== Installing dependencies ====="

cd ros_workspace

rosdep install -i --from-path src --rosdistro foxy -y

cd ../build_tool

sudo chmod u+x build-hl.sh
sudo chmod u+x install-ros.sh

   echo "${green}Now you can run build-hl.sh. Run build-hl.sh -h first for help${reset}"

