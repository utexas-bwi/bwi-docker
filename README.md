This package enables the BWI code stack on machines running Ubuntu 20.04.03 LTS+.  It creates a Docker image and container that can control a V4 BWIbot.

### Table of Contents

[Getting Started](#getting-started)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Requirements](#requirements)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Install](#install)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Build](#build)<br/>
[Usage](#usage)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Run ROS and the BWI stack in Docker](#run-ros-and-the-bwi-stack-in-docker)<br/>
[Development](#development)<br/>
[Resources](#resources)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Learning and Reference](#learning-and-reference)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Using tmux](#using-tmux-to-run-terminal-sessions-in-the-background-of-a-container)<br/>

# Getting Started

**Note that if your BWIbot already has this package setup, you can start from "Usage".**

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
## Pull the system image

todo - add instructions for finding a pre-built image

## Build a new image

From inside the `bwi-docker` directory, build the Docker image:
```
docker-compose build
```



# Setup (from host bash shell)

Add the `docker` user to the xhost users.  This enables display of UI elements and is only needed once on host startup.
```
xhost +local:docker
```

Create a project directory inside the `bwi-docker` directory.  **All docker compose commands should be executed from inside `bwi-docker` for the container to work correctly.  Failure to do so can have undesired consequesnces**.

Get your user and user group id and replace the values in the `USER` tag in `docker-compose.yml`.
```
id -u #for uid
id -g #for gid
```

# Usage
## Run ROS and the BWI stack in Docker

Run the following command to start up a detached container called `bwi_system_c` using the service `bwi_system_s` defined in `docker-compose.yml`:
```
docker-compose up -d
```

In a new terminal window, open a bash terminal inside the container we just created, `bwi_system_c`, with:
```
docker exec -ti bwi_system_c bash -l
```

In this Docker container shell session you can run ROS commands.

Run the standard [visit doors demo in AHG](https://github.com/utexas-bwi/bwi/blob/master/demo_v4.md) with the following commands.  Source your workspace with `source ~/.bashrc` if you run into any errors.
```
roslaunch bwi_launch segbot_v4_ahg.launch
```

Open another terminal in the container with `sudo docker exec -ti bwi_system_c bash -l`, and then run the AHG visit doors demo with:

```
rosrun bwi_tasks visit_door_list_smach
```

If you need to teleop the robot, use the following command inside the container:
```
rosrun segbot_bringup teleop_twist_keyboard
```

When finished, `exit` to exit the container bash session, and then stop and remove the docker resources with:
```
sudo docker-compose down
```

# Development inside the container

A development directory called `projects` persists on the host when a docker container is closed.  ROS Melodic workspaces can be added to this directory.