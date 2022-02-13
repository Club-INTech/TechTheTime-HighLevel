#!/bin/bash

echo "This is an installation of ROS2 using Debian packages. You need sudo rights to proceed. Do you want to install ROS2[Y/n]? :"

read answer

if ! [ "${answer,,}" = "y" ] || [ -z $answer ]; then
    exit 0
fi

# installing and changing locales 

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Getting gpg keys and adding ros2 repositories

sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# installing ros2:foxy

echo "Please choose dev or release installation. Release installation does not contain GUI(adapted for raspi) [D/r]? : "

while [ true ] 
do
read answer
if [ "${answer,,}" = "d" ] || [ -z $answer ]; then
    sudo apt install ros-foxy-desktop
    break
elif [ "${answer,,}" = "r" ]; then
    sudo apt install ros-foxy-ros-base
    break
else
    echo "Please choose one of two options  "
fi
done

echo "========== Installation finished ==========  "
echo "Do you want to update your bashrc to use ros2 [Y/n]? :  "

read answer

if [ "${answer,,}" = "y" ] || [ -z $answer ]; then
   echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
   source ~/.bashrc
fi

echo "Setting your ros domain id to 120  "

echo "export ROS_DOMAIN_ID=120" >> ~/.bashrc
source ~/.bashrc

echo "Installing colcon  "
sudo apt install python3-colcon-common-extensions

exit 0