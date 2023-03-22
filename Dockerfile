# use latest devel image with cudnn for all the CUDA bells n whistles
FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu18.04
SHELL ["/bin/bash", "-c"]

# set the ros version
ENV ROS_DISTRO melodic

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