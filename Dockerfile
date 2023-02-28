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

# install bwi dependencies
RUN source /opt/ros/melodic/setup.bash &&\
    rosdep update &&\
    mkdir -p tmp_ws/src &&\
    cd tmp_ws &&\
    wstool init src https://raw.githubusercontent.com/utexas-bwi/bwi/master/rosinstall/melodic.rosinstall &&\
    rosdep install --from-paths src --ignore-src --rosdistro melodic -y

# set up the bwi_knowledge_representation postgresql db
USER postgres
RUN  /etc/init.d/postgresql start &&\
    psql -c "ALTER USER postgres WITH PASSWORD 'nopass'" &&\
    createdb knowledge_base &&\
	psql -d knowledge_base -f tmp_ws/src/bwi_common/knowledge_representation/sql/schema_postgresql.sql &&\
    psql -c "SHOW data_directory;" &&\
	/etc/init.d/postgresql stop
USER $USERNAME
WORKDIR /home/$USERNAME
RUN cd tmp_ws &&\
    source /opt/ros/$ROS_DISTRO/setup.bash &&\
    catkin build utexas_ahg bwi_knowledge_representation &&\
    source devel/setup.bash &&\
    sudo /etc/init.d/postgresql start &&\
    sudo -u postgres psql -c "ALTER USER postgres WITH PASSWORD 'nopass'" &&\
    echo -e '# hostname:port:database:username:password\n\
localhost:*:knowledge_base:postgres:nopass\n' > ~/.pgpass &&\
    sudo chmod 600 ~/.pgpass &&\
    prepare_knowledge_bwi_ahg
RUN rm -r tmp_ws

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
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/amrl_msgs" >> ~/.bashrc &&\
echo ""

# copy the entrypoint into the image
COPY ./entrypoint.sh ./entrypoint.sh
# run this script on startup
ENTRYPOINT ./entrypoint.sh