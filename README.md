
## Requirements

- Ubuntu OS
- NVIDIA graphics card and drivers
- Docker
- Docker-Compose

This package is designed for use with Ubuntu OS and systems having NVIDIA graphics cards using NVIDIA drivers.  Please be sure to setup your drivers according to the article linked here under the heading [**Install Nvidia Driver Using GUI**](https://phoenixnap.com/kb/install-nvidia-drivers-ubuntu).  Do not forget to register the MOK on reboot (this option will appear in BIOS before the OS loads), otherwise the drivers will not be allowed to load.

Verify whether you have NVIDIA drivers loaded with the command `nvidia-smi`.  If you do not, it will return an error.

You might also install the CUDA toolkit with `sudo apt install nvidia-cuda-toolkit`.

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

## Run ROS and the BWI stack in Docker

Run the following command to start up the roscore process in the Docker container `bwiros`:
```
sudo docker-compose up
```

In a new terminal on the host, open a pseudo-tty bash terminal in the container `bwiros` with:
```
sudo docker exec -ti bwiros bash
```

In this psudo-tty you can run ROS commands.  Do these below to run the standard demo, visit doors in AHG.
```
roslaunch bwi_launch segbot_v4_ahg.launch
```

Open another terminal in the container and run another command in the same way:

```
sudo docker exec -ti bwiros bash
rosrun bwi_tasks visit_door_list_smach
```

If you need to teleop the robot, use the following command inside the container:
```
rosrun segbot_bringup teleop_twist_keyboard
```
