# TechTheTime-HighLevel

The code of the high level of the robot for the Erobot2022 session.

# Requirements

- ros:foxy (you can use the **install_ros.sh** script in **utils_scripts** folder with sudo permission).
- gcc (the default installation is sufficient)
- colcon (follow [this tutorial](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html#install-colcon))
- python3.7+

# Build and launch

## Build

### With the script

Run the next command to build 
```bash 
$ ./utils_scripts/build-hl.sh
```

This command will build all interfaces which are located in **ros_workspace/src/srvs_msgs**, needed for nodes communication.

Then, it will build all nodes which are located in **ros_workspace/src**

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

### Docker

It is coming

# Documentation

## Generate doc

Run the following command in **ros_workspace** folder to generate documentation:

```bash
$ doxygen
```
The generated documentation is accessible by opening **ros_workspace/doc/index.html** in your browser.