FROM osrf/ros:melodic-desktop-full
SHELL ["/bin/bash", "-c"]

ENV PATH /usr/local/cuda/bin/:$PATH
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
LABEL com.nvidia.volumes.needed="nvidia_driver"
# disable interactive terminal msgs
ENV DEBIAN_FRONTEND noninteractive

# install ros and dependencies
RUN sudo apt update
RUN sudo apt -y install apt-utils python-pip
RUN sudo apt -y install ros-melodic-desktop-full
RUN sudo apt -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
RUN sudo apt -y install libqt5websockets5-dev
RUN sudo apt -y install udev nano
RUN pip install -U pyYAML

# setup the non-root user
RUN useradd --create-home --shell /bin/bash bwilab
RUN sudo usermod --append --groups sudo,dialout bwilab
# give permission to run sudo w/ out pw prompts
RUN echo 'bwilab ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# switch user
USER bwilab
ENV ROS_DISTRO melodic
WORKDIR /home/bwilab

CMD roscore
