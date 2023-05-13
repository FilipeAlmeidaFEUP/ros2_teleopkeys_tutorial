# Flatland Teleopkeys Tutorial using ROS 2

## Setup and Pre-requisites

### ROS 2 Humble Instalation
This tutorial was built to work with ROS 2 Humble which is, at the time of writing, the latest distribution. Follow the instructions in the official [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html). 
It is recommended to at least install the desktop version, as some tools that will be used here are already contained within.

### Flatland 2

A fork of Flatland modified to work on ROS 2 natively is available on [this repository](https://github.com/JoaoCostaIFG/flatland). Follow the instalation instructions to have this version available on your machine.

### Workspace setup

To be able to build all the dependencies and run your projects, ROS needs to a designated folder known as the workspace. To setup your workspace follow the oficial [ROS 2 tutorial on how to create a workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

## Clone the repository

After everything is setup, run the following commands inside your ROS 2 workspace folder to clone the repository:

```
cd src/
git clone https://github.com/FilipeAlmeidaFEUP/ros2_teleopkeys_tutorial.git
```

## Building and installing dependencies

Before running the project for the first time, first you need to execute all these commands:

1. Resolve dependencies: `rosdep install -i --from-path src --rosdistro humble -y`
2. Build the workspace:`colcon build`
3. Source the setup script:`source install/setup.bash`

