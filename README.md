# TechTheTime-HighLevel

The high level of the robot for the Erobot2022 session.

***Note:*** The install, build and launch have been tested only for Debian and derivatives [Raspbian, Ubuntu and Lubuntu]. 

# Requirements

- ros:foxy
- rosdep
- gcc/g++ (the default installation is sufficient)
- cmake
- colcon (follow [this tutorial](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html#install-colcon))
- python3.7+
- rosdep

# Install and init

You can install all dependencies manually or you can use scripts in **build_tool/utils** directory.

After clonning the repo do:

```bash
    $ cd build_tool/utils
    $ sudo chmod u+x install-ros.sh
    $ sudo ./install-ros.sh
```
You must initialize the submodules and install dependencies with **rosdep**. You can make either by hand or using the script.

Run the following command to initialize HL:

```bash
    $ cd build_tool/utils
    $ sudo chmod u+x init-hl.sh
    $ sudo ./init-hl.sh
```

# Build and launch

## Build

### With the script

Run the next command to build 
```bash
    $ cd build_tool
    $ ./build-hl.sh [-hvr] [-p <nodes_name>]
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

## Launch

### By hand

Every time open a new terminal and source **ros_workspace/install/setup.bash**. You can now launch nodes with the following command:

```bash
    $ ros2 run <node> <executable>
```

### With the script.

Use the run script:

```bash
    $ cd build_tool
    $ sudo ./run.sh <mode> <script>
```

There is 2 modes: **basic** and **full**. There is no path-finding in the basic mode.

You can write your own scripts and put it in **script** folder. Just use the name of your script as an argument. Check out the example scripts for the syntax(available orders basically.)

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