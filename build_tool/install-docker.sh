#!/bin/bash

echo "===== Setting up environment to install docker ====="

sudo apt-get update
sudo apt-get install \
apt-transport-https \
ca-certificates \
curl \
gnupg \
lsb-release

echo "===== Finished ====="

echo "===== Adding a GPG key ====="

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

echo "===== Finished ====="

echo "===== Setting up the repository ====="

echo \
"deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
$(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

echo "===== Finished ====="

echo "===== Installing docker ====="

sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io

echo "===== Finished ====="

echo "===== Installing docker-compose ====="

sudo apt-get upgrade -y
sudo apt-get install curl python3-pip libffi-dev python-openssl libssl-dev zlib1g-dev gcc g++ make -y
sudo apt install rustc
sudo pip3 install docker-compose

echo "===== Finished ====="