FROM osrf/ros:melodic-desktop-full
SHELL ["/bin/bash", "-c"]

ENV PATH /usr/local/cuda/bin/:$PATH
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
LABEL com.nvidia.volumes.needed="nvidia_driver"

ENV DEBIAN_FRONTEND noninteractive
RUN sudo apt-get update
RUN sudo apt-get -y install apt-utils python-pip
RUN sudo apt-get -y install ros-melodic-desktop-full
RUN sudo apt-get -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

COPY ./bwiros_entrypoint.sh /
ENTRYPOINT [ "bwiros_entrypoint.sh" ]