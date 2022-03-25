# TechTheTime-HighLevel

The high level of the robot for the Erobot2022 session.

***Note:*** The install, build and launch have been tested only for Debian and derivatives [Raspbian, Ubuntu and Lubuntu]. 

# Requirements

- ros:foxy.
- gcc/g++ (the default installation is sufficient)
- cmake
- colcon (follow [this tutorial](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html#install-colcon))
- python3.7+
- rosdep

# Install

You can install all dependencies manually or you can use bash script in **build_tool** directory.

After clonning the repo do:

```bash
    $ cd build_tool
    $ sudo chmod u+x init-hl.sh
    $ sudo ./init-hl.sh
```

The choose the appropriate options. You can either build from source and run the nodes manually or use 
a docker-compose.

# Build and launch

## Build

### With the script

Run the next command to build 
```bash
    $ cd build_tool
    $ ./build-hl.sh [-hv] [-p <nodes_name>]
```

***Note:*** Use **-h** options for help.

This command will build all interfaces which are located in **ros_workspace/src/srvs_msgs**, needed for nodes communication.

Then, it will build all available or all provided nodes which are located in **ros_workspace/src**

### From source

You need **colcon** to build from source. 

Place you in **ros_workspace/src/srvs_msgs** to build interfaces:

```bash
$ cd ros_workspace/src/srvs_msgs
$ colcon build [--packages-select] [<your_interface>]
$ source install/setup.bash
```

Place you in **ros_workspace** to build nodes:

```bash
$ cd ros_workspace
$ colcon build [--packages-select] [<your_node>]
$ source install/setup.bash
```

***Note:*** You must build an interface and source its installation before you build the node which uses this interface.

### Docker

You can build an image manually or by running docker-compose, which will also run the HL.

***Note:*** Be careful and delete all build, install and log directories after manual build. If not you risk to use cmake cache, which uses absolute path incompatible with container's one.

You can build image manually by running:

```bash
    $ sudo docker build .
```

Run the following if you want to delete this image: 

```bash
    $ sudo docker rmi techthetime-highlevel --force
    $ sudo docker system prune 
```

## Launch

### By hand

Every time open a new terminal and source **ros_workspace/install/setup.bash**. You can now launch nodes with the following command:

```bash
$ ros2 run <node> <executable>
```

### Docker

You have two ways of running HL with docker.
The simplest way is to build and run all at the same time:

```bash
    $ sudo docker-compose up
```

If you have built an image manually and you are familiar with docker, you can run a container an execute your own 
commands like this:

```bash
    $ sudo docker run techthetime-highlevel [COMMAND] [ARG...]
```

You can read about this command and docker [there](https://docs.docker.com/engine/reference/commandline/run/)

# Documentation

## GitHub pages

A github action was added, so the documentation is generated and published automatically.

Check documentation there https://club-intech.github.io/TechTheTime-HighLevel/

## Generate doc

You must install [doxygen](https://www.doxygen.nl/manual/install.html) in order to generate the documentation locally.

***Note:*** You must install flex and bison to build doxygen from source.

Run the following command in root folder to generate documentation:

```bash
$ doxygen
```
The generated documentation is accessible by opening **html/index.html** in your browser.