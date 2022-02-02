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

# setup the non-root user
RUN useradd --create-home --shell /bin/bash bwilab
RUN sudo usermod --append --groups sudo,dialout bwilab
# give permission to run sudo w/ out pw prompts
RUN echo 'bwilab ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
# switch user
USER bwilab

# RUN rosdep init
RUN rosdep update

WORKDIR /home/bwilab
ENV ROS_DISTRO melodic
RUN source /opt/ros/$ROS_DISTRO/setup.bash
RUN mkdir -p catkin_ws/src
WORKDIR /home/bwilab/catkin_ws
RUN wstool init src https://raw.githubusercontent.com/utexas-bwi/bwi/master/rosinstall/$ROS_DISTRO.rosinstall
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

WORKDIR /home/bwilab/catkin_ws/src/bwi_common
RUN git submodule init
RUN git submodule update

RUN pip install -U pyYAML

WORKDIR /home/bwilab/catkin_ws/src
RUN git clone --branch ahg2s_map https://github.com/utexas-bwi/ahg_common.git
WORKDIR /home/bwilab/catkin_ws
RUN catkin config --extend /opt/ros/$ROS_DISTRO
RUN catkin build -j6

WORKDIR /home/bwilab
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /home/bwilab/.bashrc
RUN echo 'source /home/bwilab/catkin_ws/devel/setup.bash' >> /home/bwilab/.bashrc
RUN echo \
'export SEGWAY_INTERFACE_ADDRESS=10.66.171.1 \n\
export SEGWAY_IP_ADDRESS=10.66.171.5 \n\
export SEGWAY_IP_PORT_NUM=8080 \n\
export SEGWAY_BASE_PLATFORM=RMP_110 \n\
export SEGWAY_PLATFORM_NAME=RMP_110' \
    >> /home/bwilab/.bashrc

CMD roscore