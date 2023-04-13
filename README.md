This package enables the BWI code stack on machines running Ubuntu 20.04.03 LTS+.  It creates a Docker image and container that can control a BWIbot V2 or V4.

### Table of Contents

[Getting Started](#getting-started)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Requirements](#requirements)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Install](#install)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Setup](#setup)<br/>
[Usage](#usage)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Run ROS and the BWI codebase](#run-ros-and-the-bwi-codebase)<br/>
&nbsp;&nbsp;&nbsp;&nbsp;[Development](#development-inside-the-container)<br/>
[Build a New Image](#build-a-new-image-not-necessary-on-configured-robots)<br/>

# Getting Started

**Note that if you are working on a lab BWIbot the requirements are already met.**

## Requirements

- Ubuntu OS
- NVIDIA graphics card and drivers
- [Docker](https://docs.docker.com/engine/install/ubuntu/)
- Docker Compose (installed with Docker document above)
- [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

This package is designed for use with Ubuntu OS and systems having NVIDIA graphics cards using NVIDIA drivers.  Verify whether you have NVIDIA drivers loaded with the command `nvidia-smi`.  If you do not, it will return an error.  This article describes a verified method of installing Nvidia drivers on Ubuntu under the heading [Install Nvidia Driver Using GUI](https://phoenixnap.com/kb/install-nvidia-drivers-ubuntu).  You will need to reboot for changes to take effect.

## Install

Clone the repo into a useful directory, eg `~/<your user>/`.
```
git clone https://github.com/utexas-bwi/bwi-docker.git
```
The rest of this setup assumes there is already a docker image called `bwi_system_i` on your system.  If you are installing on a new robot or computer, you will need to build the image before proceeding.  For image build instructions, see the bottom of this page.


## Setup

1. Copy bash tools to your user space.
From the `bwi-docker` directory on the host, copy the `bash_tools` to your userspace with
```
echo "source `pwd`/bash_tools" >> ~/.bashrc
source ~/.bashrc
```

1. Copy the correct config for your robot and put it in the main project directory.
From the `bwi-docker` directory, run the command below.  Replace `<robot version>` with `V2` for running on a Bwibot V2 and `V4` for a BWIbot V4.  If installing on a standalone computer, you can use `computer` in place of `<robot_version`>:
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

Before starting a container, you can add sourcing of an existing catkin workspace by updating the `$WORKSPACE` variable.  Provide the relative path only from inside the `bwi-docker/projects` directory.  Presently only one workspace can be added this way:
```
bwi-ws <workspace_directory>
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


## Run ROS and the BWI codebase

In the Docker container shell you can run ROS commands.

### Install the BWI code base

On the host, make a catkin_ws under the "projects" directory:
```
cd projects && mkdir -p catkin_ws/src
cd catkin_ws/src
```
Install the BWI packages with vcstool:
```
vcs import --recursive --input https://raw.githubusercontent.com/utexas-bwi/bwi/master/rosinstall/melodic.rosinstall
```

Start a container and open a shell in it:
```
bwi-start
bwi-shell
```
Build the packages inside the container shell:
```
catkin build
```
Don't forget to source your workspace before running packages.:
```
source devel/setup.bash
```
Note: on a V4 robot, you will additionally do `sv4` after the previous step to load V4 environment variables.

### Run a BWI hallway demo

Run the standard visit doors demo in AHG.
- [V2 Demo](https://github.com/utexas-bwi/bwi/blob/master/demo_v2.md)
- [V4 Demo](https://github.com/utexas-bwi/bwi/blob/master/demo_v4.md)

Commands should be run inside a container shell, which you can open in a running continer with
```
bwi-shell
```

When finished, `exit` to exit the container bash session, and then stop and remove the docker resources with:
```
bwi-stop
```

## Development inside the container

A development directory called `projects` persists on the host when a docker container is closed.  ROS Melodic workspaces can be added to this directory and built from inside the container.  Their contents will persist after the container is closed.

It is best to manage files from the host, and use the container for building and running code.  This is why you are instructed to use `vcstool` to clone git repositories onto the host machine, rather than inside the container.

# Build a new image (not necessary on configured robots)

From inside the `bwi-docker` directory, build the Docker image:
```
docker compose build

# or if bash tools have been setup (see Setup)
bwi-build
```

The visit doors demo relies on a postgres SQL server and database.  This is preinstalled on configured robots, but on new machines, you will need to set them up using [postgres_docker](https://github.com/utexas-bwi/postgres_docker).
