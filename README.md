This package enables the BWI code stack on machines running Ubuntu 20.04.03 LTS+.  It creates a Docker image and container that can control a V4 BWIbot.

### Table of Contents

[Getting Started](#getting-started)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Requirements](#requirements)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Install](#install)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Setup](#setup)<br/>
[Usage](#usage)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Run ROS and the BWI stack in Docker](#run-ros-and-the-bwi-stack-in-docker)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Development](#development-inside-the-container)<br/>

# Getting Started

**Note that if your BWIbot already has this package installed, you can start from "Setup".**

## Requirements

- Ubuntu OS
- NVIDIA graphics card and drivers
- [Docker](https://docs.docker.com/engine/install/ubuntu/)
- [Docker-Compose](https://docs.docker.com/compose/install/)
- [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

This package is designed for use with Ubuntu OS and systems having NVIDIA graphics cards using NVIDIA drivers.  Verify whether you have NVIDIA drivers loaded with the command `nvidia-smi`.  If you do not, it will return an error.  This article describes a verified method of installing Nvidia drivers on Ubuntu under the heading [Install Nvidia Driver Using GUI](https://phoenixnap.com/kb/install-nvidia-drivers-ubuntu).  Do not forget to register the MOK on reboot (this option will appear in BIOS before the OS loads), otherwise the drivers will not be allowed to load.

## Install

Clone the repo into a useful directory, eg `/home/<your user>/`.  Checkout the "system_only" branch.
```
git clone https://github.com/utexas-bwi/bwi-docker.git
git checkout system_only
```
### Pull the system image

todo - add instructions for finding a pre-built image

### Build a new image (not necessary on configured robots)

From inside the `bwi-docker` directory, build the Docker image:
```
docker compose build

# or if bash tools have been setup (see Setup)
bwi-build
```

## Setup

From the `bwi-docker` directory on the host, copy the `bash_tools` to your userspace with
```
echo "source `pwd`/bash_tools" >> ~/.bashrc
source ~/.bashrc
```

Next, edit `docker-compose.yml` with the settings for your robot.  To run the container on an actual robot, you need to ensure some `devices` are accessible to the container.  Uncomment the lines as below for either a v2 or a v4+ robot.  To make additional usb devices available to the container, add them here.
```
# For a v2 BWIbot:
    devices:
      - /dev/hokuyo:/dev/hokuyo
      - /dev/ttyUSB0:/dev/ttyUSB0
      # - /dev/segway_rmp:/dev/segway_rmp
      - /dev/kinect:/dev/kinect
      

# For a v4+ BWIbot:
    devices:
       - /dev/hokuyo:/dev/hokuyo
       # - /dev/ttyUSB0:/dev/ttyUSB0
       - /dev/segway_rmp:/dev/segway_rmp
       - /dev/kinect:/dev/kinect

```

Lastly, when running a BWIbot V2, change the `env_file` variable in `docker-compose.yml` to:
```
env_file: base_env/v2_env
```

**Docker (bwi-*) commands should be executed from inside `bwi-docker` for the container to work correctly.  Not doing so can have undesired consequesnces**.

Start the docker container with
```
bwi-start
```
From a new host shell, open a bash shell inside the container.  **`bwi-shell` is safe to run from any directory on the host**
```
bwi-shell
```
From inside the container, setup a catkin_ws [as usual](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) and clone the bwi repo into `catkin_ws/src`.

# Usage

Before starting a container, you can add workspace sourcing by updating the `$WORKSPACE` variable.  Note that the path should be fully defined, not using shorthand like `~`, because they are interpreted as host locations.  Presently only one workspace can be added this way, but you can always source additional workspaces by adding them to ~/.bashrc from inside the container.  However, the settings will not persist after the container is stopped.
```
bwi-ws /home/bwi-docker/projects/<workspace_directory>
```
Start the docker container with
```
bwi-start
```
From a new host shell, open a bash shell inside the container with
```
bwi-shell
```
When finished, remove the container with
```
bwi-stop
```

## Run ROS and the BWI stack in Docker

In the Docker container shell you can run ROS commands.

Run the standard [visit doors demo in AHG](https://github.com/utexas-bwi/bwi/blob/master/demo_v4.md) with the following commands.  Be sure you are running the launch file for the correct robot - either v4 or v2.
```
roslaunch bwi_launch segbot_v4_ahg.launch
```

Open another terminal in the container with `bwidocker shell`, and then run the AHG visit doors demo with:

```
rosrun bwi_tasks visit_door_list_smach
```

If you need to teleop the robot, use the following command inside the container:
```
rosrun segbot_bringup teleop_twist_keyboard
```

When finished, `exit` to exit the container bash session, and then stop and remove the docker resources in the terminal you started the container with:
```
(cancel the process with "ctrl + c")
bwi-stop
```

## Development inside the container

A development directory called `projects` persists on the host when a docker container is closed.  ROS Melodic workspaces can be added to this directory and built from inside the container.  Their contents will persist after the container is closed.