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

Clone the repo into a useful directory, eg `/home/<user>/`.
```
git clone https://github.com/utexas-bwi/bwi-docker.git
```

## Build

From inside the `bwi-docker` directory, build the Docker image:
```
sudo docker-compose build
```

Add the `docker` user to the xhost users (this enables display of UI elements)
```
xhost +local:docker
```

# Usage
## Run ROS and the BWI stack in Docker

Run the following command to start up a detached container called `bwibase_c` using the service `bwibase_s` defined in `docker-compose.yml`:
```
sudo docker-compose run -d --name bwibase_c bwibase_s
```

In a new terminal window, open a bash terminal inside the container we just created, `bwibase_c`, with:
```
sudo docker exec -ti bwibase_c bash
```

In this Docker container shell session you can run ROS commands.

For simplicity, the remaining instructions assume using Docker commands to open new shell sessions in the container for each process.  However, this is where using `tmux` can be really handy.  See the [Resources](#resources) section below.

Run the standard [visit doors demo in AHG](https://github.com/utexas-bwi/bwi/blob/master/demo_v4.md) with the following commands.  Source your workspace with `source ~/.bashrc` if you run into any errors.
```
roslaunch bwi_launch segbot_v4_ahg.launch
```

Open another terminal in the container with `sudo docker exec -ti bwibase_c bash`, and then run the AHG visit doors demo with:

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

# Development

In order to test new packages on the BWI code base, a development workspace, `dev_ws`, is created in `volumes/` that persists on the host when a docker container is closed.  This workspace must be configured before it can be used.

First, you may need to change the permissions on the directory on the host bc it was created with root permissions by docker.  From the main `bwi-docker` directory, run the command below.  The `<username>` and `<groupname>` are both the name of your user on the host machine, ie `bwiuser:bwiuser`.  BE AWARE THAT `chown -R` CHANGES PERMISSIONS FOR EVERYTHING IN VOLUMES because of the `-R` flag.  If you add other volumes that need different permissions, be sure not to change them recursively.
```
sudo chown -R <username>:<groupname> volumes
```
Next navigate to the `dev_ws` **inside the container** to source the main workspace before building your development workspace.  This ensures you are including all the settings from the main `catkin_ws`.
```
cd /home/bwilab/dev_ws
mkdir src
source ~/.bashrc
catkin build
```
If it builds correctly, you will see new directores `build` and `devel`.  Then when you want to use the `dev_ws`, source it with
```
source /home/bwilab/dev_ws/devel/setup.bash
```
____________

# Resources

## Learning and Reference
- [LinkedIn Learning Docker Course](https://www.linkedin.com/learning-login/share?account=36306084&forceAccount=false&redirect=https%3A%2F%2Fwww.linkedin.com%2Flearning%2Flearning-docker-2018%3Ftrk%3Dshare_ent_url%26shareId%3D%252F%252FR0%252F9JHQI2Iyed65k0LzQ%253D%253D) - free to UT members.
- [Docker Reference](https://docs.docker.com/reference/)


## Using tmux to run multiple terminal sessions in a container


**tmux** is a great tool for working in terminal, especially with a remote host like ssh or a docker container.  It allows users to run multiple shell sessions in the background or in panes, rather than opening a new terminal on your host and logging in to a shell session on the remote every time you want to run a new process.

Start a tmux session by typing `tmux` to start a numbered session, or give it a name with:
```
tmux new-session -sSessionName
```

Attach to a session with:
```
tmux attach-session -t SessionName
```

Keyboard shortcuts allow you to open new windows inside a tmux session and switch between them:

- **ctrl+b, c** allows you to **c**reate a new window in a tmux session.
- **ctrl+b, n** to switch to the the **n**ext window in a session.
- **ctrl+b, p** to switch to the **p**revious window.
- **ctrl+b, w** gives a selectable list of all windows in a session.

To close a window, simply type
```
tmux kill-window -t <window number>
```

Panes allow you to split a window into sections:
- **ctrl+b, %** splits vertically.
- **ctrl+b, "** splits horizontally.
- **ctrl+b, \<arrow keys\>** to switch between panes.

To close a current pane, simply type
```
tmux kill-pane
```

To close the entire session:

`tmux kill-session` to kill the current session, or

`tmux kill-session -t <session name>` to kill another session by target name (or id).

Plenty of `tmux` cheat sheets and guides are available online for more info.
