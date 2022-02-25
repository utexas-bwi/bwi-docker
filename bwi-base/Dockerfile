FROM osrf/ros:melodic-desktop-full
SHELL ["/bin/bash", "-c"]

# settings that enable container use of display
ENV PATH /usr/local/cuda/bin/:$PATH
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
LABEL com.nvidia.volumes.needed="nvidia_driver"

# set non-interactive shell for all this installation
ENV DEBIAN_FRONTEND noninteractive

# set the ros version
ENV ROS_DISTRO melodic
# update install resources to latest
RUN sudo apt-get update

# setup the non-root user
RUN useradd --create-home --shell /bin/bash bwilab
RUN sudo usermod --append --groups sudo,dialout bwilab
# give permission to run sudo w/ out pw prompts
RUN echo 'bwilab ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
# switch user
USER bwilab

# create a ROS catkin_ws
WORKDIR /home/bwilab
RUN mkdir -p catkin_ws/src
WORKDIR /home/bwilab/catkin_ws

# these ROS commands must be executed in the same "RUN" process as a "source" command,
# or use `RUN /ros_entrypoint.sh command`
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    wstool init src https://raw.githubusercontent.com/utexas-bwi/bwi/master/rosinstall/$ROS_DISTRO.rosinstall &&\
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep update &&\
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
# make sure the repo submodules are installed
WORKDIR /home/bwilab/catkin_ws/src/bwi_common && \
    git submodule init &&\
    git submodule update
# get a missing dependency for the database
RUN pip install -U pyYAML
# install some more new repos (TODO- add these to melodic-docker.rosinstall)
WORKDIR /home/bwilab/catkin_ws/src
RUN git clone --branch ahg2s_map https://github.com/utexas-bwi/ahg_common.git
RUN git clone https://github.com/utexas-bwi/bwi_robofleet.git
# finally build the bwi code base
WORKDIR /home/bwilab/catkin_ws
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
RUN  /etc/init.d/postgresql start &&\
    psql -c "ALTER USER postgres WITH PASSWORD 'nopass'" &&\
    createdb knowledge_base &&\
	psql -d knowledge_base -f /home/bwilab/catkin_ws/src/bwi_common/knowledge_representation/sql/schema_postgresql.sql &&\
	/etc/init.d/postgresql stop

# create a mount point for the db
VOLUME /var/lib/postgresql
# create a mount point for the catkin_ws
VOLUME /home/bwilab/catkin_ws

# copy the entrypoint into the image
COPY ./bwibase_entrypoint.sh /
# run this script on startup
ENTRYPOINT [bwibase_entrypoint.sh]