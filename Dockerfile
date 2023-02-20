# use latest devel image with cudnn for all the CUDA bells n whistles
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu18.04
SHELL ["/bin/bash", "-c"]

# set the ros version
ENV ROS_DISTRO melodic

RUN apt-get update &&\
    apt-get -y install apt-utils curl nano vim tmux python-pip

RUN echo 'deb http://packages.ros.org/ros/ubuntu bionic main' > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# update install resources to latest
# set non-interactive shell for all this installation
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y install \
    ros-${ROS_DISTRO}-desktop-full python-rosdep python-rosinstall \
    python-rosinstall-generator python-wstool build-essential \
    libqt5websockets5-dev qt5-default && \
    pip install -U pyYAML
RUN rosdep init


ENV USERNAME bwi-docker
# setup the non-root user
RUN useradd -m -s /bin/bash -G sudo,dialout $USERNAME
USER $USERNAME
WORKDIR /home/$USERNAME
RUN rosdep update
RUN git clone https://github.com/ut-amrl/amrl_msgs.git
RUN source /opt/ros/melodic/setup.bash && cd amrl_msgs && export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH && make -j

# setup .bashrc
SHELL ["/bin/bash", "-l", "-c"]

RUN echo -e '\
source /opt/ros/melodic/setup.bash\n\
# source /home/${USERNAME}/catkin_ws/devel/setup.bash\n\
export SEGWAY_INTERFACE_ADDRESS=10.66.171.1\n\
export SEGWAY_IP_ADDRESS=10.66.171.5\n\
export SEGWAY_IP_PORT_NUM=8080\n\
export SEGWAY_BASE_PLATFORM=RMP_110\n\
export SEGWAY_PLATFORM_NAME=RMP_110\n\
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_msgs' >> ~/.profile

RUN echo -e '\
source /opt/ros/melodic/setup.bash\n\
# source /home/${USERNAME}/catkin_ws/devel/setup.bash\n\
export SEGWAY_INTERFACE_ADDRESS=10.66.171.1\n\
export SEGWAY_IP_ADDRESS=10.66.171.5\n\
export SEGWAY_IP_PORT_NUM=8080\n\
export SEGWAY_BASE_PLATFORM=RMP_110\n\
export SEGWAY_PLATFORM_NAME=RMP_110\n\
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_msgs' >> ~/.bashrc

# ARG HOST_UID
# ARG HOST_GID

# USER root
# ENV host_uid=${HOST_UID}
# ENV host_gid=${HOST_GID}

# copy the entrypoint into the image
COPY ./bwibase_entrypoint.sh ./bwibase_entrypoint.sh
# run this script on startup
# ENTRYPOINT ./bwibase_entrypoint.sh -u $host_uid -g $host_gid
ENTRYPOINT ./bwibase_entrypoint.sh