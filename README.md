This package enables the BWI code stack on machines running Ubuntu 20.04.03 LTS+.  It creates a Docker image and container that can control a BWIbot V2 or V4.

### Table of Contents

[Getting Started](#getting-started)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Requirements](#requirements)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Install](#install)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Setup](#setup)<br/>
[Usage](#usage)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Run ROS and the BWI codebase](#run-ros-and-the-bwi-codebase)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Development](#development-inside-the-container)<br/>

# Getting Started

**Note that if you are working on a lab BWIbot the requirements are already met.**

## Requirements

- Ubuntu OS
- NVIDIA graphics card and drivers
- [Docker](https://docs.docker.com/engine/install/ubuntu/)
- Docker Compose v2 Plugin (installed with Docker document above)
- [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

This package is designed for use with Ubuntu OS and systems having NVIDIA graphics cards using NVIDIA drivers.  Verify whether you have NVIDIA drivers loaded with the command `nvidia-smi`.  If you do not, it will return an error.  This article describes a verified method of installing Nvidia drivers on Ubuntu under the heading [Install Nvidia Driver Using GUI](https://phoenixnap.com/kb/install-nvidia-drivers-ubuntu).  Do not forget to register the MOK on reboot (this option will appear in BIOS before the OS loads), otherwise the drivers will not be allowed to load.

## Install

Clone the repo into a useful directory, eg `~/<your user>/`.  Checkout the "system_only" branch.
```
git clone --branch system_only https://github.com/utexas-bwi/bwi-docker.git
```
The rest of this setup assumes there is already a docker image called `bwi_system_i` on your system.  For build instructions, see the bottom of this page.


## Setup

1. Copy bash tools to your user space.
From the `bwi-docker` directory on the host, copy the `bash_tools` to your userspace with
```
echo "source `pwd`/bash_tools" >> ~/.bashrc
source ~/.bashrc
```

1. Copy the correct config for your robot and put it in the main project directory.
From the `bwi-docker` directory, run the command below.  Replace `<robot version>` with `V2` for running on a Bwibot V2 and `V4` for a BWIbot V4.:
```
cp docker_configs/docker-compose_<robot version>.yml docker-compose.yml
```

# Usage

These commands should be run from the host:

| command | operation | context |
| --- | --- | --- |
| `bwi-start` | Start the docker container | **Run from project directory `bwi-docker`** |
| `bwi-shell` | Open a bash shell inside the container | Run from anywhere on the host |
| `bwi-stop` | Stop and remove the docker container | **Run from project directory `bwi-docker`** |

Optional commands: 

| command | operation | context |
| --- | --- | --- |
| `bwi-ws` | Set a container workspace path to source (see below) | Run from anywhere on host |
| `bwi-build ` | Build the container image (admin only) | **Run from project directory `bwi-docker`** |


Source a workspace from the image ~/.bashrc and ~/.profile:

Before starting a container, you can add sourcing of an existing catkin workspace by updating the `$WORKSPACE` variable.  Provide the relative path only from inside the `bwi-docker` directory.  Presently only one workspace can be added this way:
```
bwi-ws projects/<workspace_directory>
```
Clear this value with
```
bwi-ws clear
```

## Development and Persistent Data

Only changes under the following directories will persist after a container is closed.

| directory | purpose and usage |
| --- | --- |
| `projects` | **put catkin workspaces and other dev files in this directory** |
| `base_env` | a place for keeping robot-specific environment variables |
| `knowledge_db` | do not edit - the database files for `bwi_knowledge_representation`, which is managed by postgres |


## Run ROS and the BWI codebase

In the Docker container shell you can run ROS commands.

### Install the BWI code base

Start a container and open a shell in it:
```
bwi-start
bwi-shell
```

Make a catkin_ws under the "projects" directory:
```
cd projects && mkdir -p catkin_ws/src
cd catkin_ws
```
Install the BWI packages with wstool:
```
wstool init src https://raw.githubusercontent.com/utexas-bwi/bwi/master/rosinstall/$ROS_DISTRO.rosinstall
```
Build the packages with catkin build:
```
catkin build
```
Don't forget to source your workspace before running packages.:
```
source devel/setup.bash
```

### Run a BWI hallway demo

Run the standard visit doors demo in AHG.
- [V2 Demo](https://github.com/utexas-bwi/bwi/blob/master/demo_v2.md)
- [V4 Demo](https://github.com/utexas-bwi/bwi/blob/master/demo_v4.md)

Commands should be run in a shell inside the continer.  which you can open in a running continer with
```
bwi-shell
```

When finished, `exit` to exit the container bash session, and then stop and remove the docker resources with:
```
bwi-stop
```

## Development inside the container

A development directory called `projects` persists on the host when a docker container is closed.  ROS Melodic workspaces can be added to this directory and built from inside the container.  Their contents will persist after the container is closed.

### Build a new image (not necessary on configured robots)

From inside the `bwi-docker` directory, build the Docker image:
```
docker compose build

# or if bash tools have been setup (see Setup)
bwi-build
```
