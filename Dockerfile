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

# RUN useradd -m -s /bin/bash -G sudo,dialout $USERNAME &&\
#     echo '${USERNAME} ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# setup the non-root user
ENV USERNAME bwi-docker
RUN useradd --create-home --shell /bin/bash ${USERNAME} &&\
    usermod --append --groups sudo,dialout ${USERNAME} &&\
    # give permission to run sudo w/ out pw prompts
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USERNAME
WORKDIR /home/$USERNAME

# ! --- not the best method to install depends since we are not keeping this ws, but it works
RUN source /opt/ros/melodic/setup.bash &&\
    rosdep update &&\
    mkdir -p catkin_ws/src &&\
    cd catkin_ws &&\
    wstool init src https://raw.githubusercontent.com/utexas-bwi/bwi/master/rosinstall/melodic_docker.rosinstall &&\
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# set the password for postgresql db
USER postgres
RUN  /etc/init.d/postgresql start &&\
    psql -c "ALTER USER postgres WITH PASSWORD 'nopass'" &&\
    createdb knowledge_base &&\
	psql -d knowledge_base -f catkin_ws/src/bwi_common/knowledge_representation/sql/schema_postgresql.sql &&\
	/etc/init.d/postgresql stop

USER $USERNAME
WORKDIR /home/$USERNAME
# ! --- be careful not to delete your data
RUN rm -r catkin_ws
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
COPY ./entrypoint.sh ./entrypoint.sh
# run this script on startup
ENTRYPOINT ./entrypoint.sh