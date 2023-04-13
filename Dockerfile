# use latest devel image with cudnn for all the CUDA bells n whistles
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu18.04
SHELL ["/bin/bash", "-c"]

# set the ros version
ENV ROS_DISTRO melodic
ENV UDEV=1

RUN apt-get update &&\
    apt-get -y install apt-utils curl nano vim tmux python-pip \
    && echo 'deb http://packages.ros.org/ros/ubuntu bionic main' > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# update install resources to latest
# set non-interactive shell for all this installation
RUN apt-get update &&\
    DEBIAN_FRONTEND=noninteractive apt-get -y install \
    ros-${ROS_DISTRO}-desktop-full python-rosdep python-rosinstall \
    python-rosinstall-generator python-wstool build-essential \
    python-catkin-tools libqt5websockets5-dev qt5-default &&\
    pip install -U pyYAML &&\
    rosdep init

# switch to home dir
WORKDIR /root

# install bwi dependencies
RUN source /opt/ros/melodic/setup.bash &&\
    rosdep update &&\
    mkdir -p tmp_ws/src &&\
    cd tmp_ws &&\
    wstool init src https://raw.githubusercontent.com/utexas-bwi/bwi/master/rosinstall/melodic.rosinstall &&\
    rosdep install --from-paths src --ignore-src --rosdistro melodic -y &&\
    cd /root && rm -r tmp_ws

# setup amrl messages
RUN git clone https://github.com/ut-amrl/amrl_msgs.git
RUN source /opt/ros/melodic/setup.bash && cd amrl_msgs && export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH && make -j

#Azure Kinect 
ARG DEBIAN_FRONTEND=noninteractive
RUN echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | sudo debconf-set-selections
RUN curl https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
RUN echo "deb [arch=amd64] https://packages.microsoft.com/ubuntu/18.04/prod bionic main" >> /etc/apt/sources.list.d/azure_kinect.list
RUN apt-get update
RUN apt-get -y install wget
RUN apt-get -y install libk4a1.4 libk4a1.4-dev k4a-tools
RUN wget https://raw.githubusercontent.com/microsoft/Azure-Kinect-Sensor-SDK/develop/scripts/99-k4a.rules -O /etc/udev/rules.d/99-k4a.rules

RUN apt-get update
RUN apt-get -y install software-properties-common

#https://raw.githubusercontent.com/chakio/azure_kinect_ros/devel/Dockerfile
#######################################################################
##                       install nvidia docker                       ##
#######################################################################

RUN apt-get update && apt-get install -y --no-install-recommends \
    libxau-dev \
    libxdmcp-dev \
    libxcb1-dev \
    libxext-dev \
    libx11-dev

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# install GLX-Gears
RUN apt update && apt install -y --no-install-recommends \
    mesa-utils x11-apps

#######################################################################
##                       install azure kinect                        ##
#######################################################################

# Configuring the repositories
RUN curl https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
RUN apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
RUN apt-get update && apt-get install -y \
    ninja-build \
    doxygen \
    clang \
    gcc-multilib-arm-linux-gnueabihf \
    g++-multilib-arm-linux-gnueabihf && \
   rm -rf /var/lib/apt/lists/*
RUN add-apt-repository -y ppa:ubuntu-x-swat/updates
RUN apt-get update && apt-get install -y \
    freeglut3 \
    freeglut3-dev \
    libgl1-mesa-dev \
    mesa-common-dev \
    libsoundio-dev \
    libvulkan-dev \
    libxcursor-dev \
    libxinerama-dev \
    libxi-dev \
    libxrandr-dev \
    uuid-dev \
    libsdl2-dev \
    usbutils \
    libusb-1.0-0-dev \
    openssl \
    libssl-dev \
    mesa-utils \
    x11-apps && \
    rm -rf /var/lib/apt/lists/*

    

#######################################################################
##                    support for kinect_wrapper                     ##
#######################################################################

RUN apt-get update && apt-get install -y libgtk-3-dev libceres-dev ddd

# setup .bashrc
SHELL ["/bin/bash", "-l", "-c"]
RUN echo -e "\
source /opt/ros/melodic/setup.bash\n\
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_msgs" >> ~/.profile \
&& echo -e "\
source /opt/ros/melodic/setup.bash\n\
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_msgs" >> ~/.bashrc

# copy the entrypoint into the image
COPY ./entrypoint.sh /entrypoint.sh
# run this script on startup
ENTRYPOINT /entrypoint.sh
