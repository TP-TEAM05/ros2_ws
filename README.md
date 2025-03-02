# ROS2 Controller repository
ROS2 Workspace for car controller

## Description
- Tested with ROS2 Humble [Docs](https://docs.ros.org/en/humble/index.html)
- Tested on Raspberry Pi 4 with Ubuntu Server 22.04

## Setup Instructions
Setup instructions for using and developing car controller.

### Prerequisities
- ROS2 Humble is already setup. [Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Library `nlohmann-json3-dev` is installed
- Library `rpi.gpio` is installed
- Library `libcurl4-openssl-dev` is installed
- Library `libboost-all-dev` is installed

### Steps

- Clone the repository `git clone git@github.com:ReCoFIIT/ros2_ws.git ~/ros2_ws`

- Go to the workspace folder `cd ~/ros2_ws`

- Pre-build the ROS2 workspace. This will create workspace files. Now run `source ~/ros2_ws/install/setup.bash`.

- Build the ROS2. `colcon build`
