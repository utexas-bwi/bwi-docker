FROM osrf/ros:melodic-desktop-full
SHELL ["/bin/bash", "-c"]

# settings that enable container use of display
ENV PATH /usr/local/cuda/bin/:$PATH
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
LABEL com.nvidia.volumes.needed="nvidia_driver"

# set non-interactive shell for all this installation
ENV DEBIAN_FRONTEND noninteractive

# setup the non-root user
RUN useradd --create-home --shell /bin/bash bwilab &&\
    usermod --append --groups sudo,dialout bwilab &&\
    # give permission to run sudo w/ out pw prompts
    echo 'bwilab ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
# switch user
USER bwilab

# update install resources to latest
RUN sudo apt-get update &&\
    sudo apt-get -y install apt-utils python-pip nano vim tmux \
    ros-melodic-desktop-full python-rosdep python-rosinstall \
    python-rosinstall-generator python-wstool build-essential \
    libqt5websockets5-dev qt5-default && \
    pip install -U pyYAML

# create a ROS catkin_ws
WORKDIR /home/bwilab
RUN mkdir -p catkin_ws/src
ENV WORKSPACE /home/bwilab/catkin_ws
WORKDIR $WORKSPACE
# set the ros version
ENV ROS_DISTRO melodic

# these ROS commands must be executed in the same "RUN" process as a "source" command,
# or use `RUN /ros_entrypoint.sh command`
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    wstool init src https://raw.githubusercontent.com/utexas-bwi/bwi/master/rosinstall/melodic_docker.rosinstall &&\
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep update &&\
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# finally build the bwi code base
WORKDIR $WORKSPACE
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    catkin build

# set some more env variables, bc they are changed after "catkin build"
# these are set again in the entrypoint
WORKDIR /home/bwilab
ENV SEGWAY_INTERFACE_ADDRESS=10.66.171.1
ENV SEGWAY_IP_ADDRESS=10.66.171.5
ENV SEGWAY_IP_PORT_NUM=8080
ENV SEGWAY_BASE_PLATFORM=RMP_110
ENV SEGWAY_PLATFORM_NAME=RMP_110

# set the password for postgresql db
USER postgres
RUN  /etc/init.d/postgresql start &&\
    psql -c "ALTER USER postgres WITH PASSWORD 'nopass'" &&\
    createdb knowledge_base &&\
	psql -d knowledge_base -f $WORKSPACE/src/bwi_common/knowledge_representation/sql/schema_postgresql.sql &&\
	/etc/init.d/postgresql stop

USER bwilab

# update bashrc (warn: dockerfile ENV variables do not work in echo text)
RUN echo -e '\
source /opt/ros/melodic/setup.bash\n\
source /home/bwilab/catkin_ws/devel/setup.bash\n\
export SEGWAY_INTERFACE_ADDRESS=10.66.171.1\n\
export SEGWAY_IP_ADDRESS=10.66.171.5\n\
export SEGWAY_IP_PORT_NUM=8080\n\
export SEGWAY_BASE_PLATFORM=RMP_110\n\
export SEGWAY_PLATFORM_NAME=RMP_110' >> ~/.bashrc

# copy the entrypoint into the image
COPY ./bwibase_entrypoint.sh /bwibase_entrypoint.sh
# run this script on startup
ENTRYPOINT ["/bwibase_entrypoint.sh"]
