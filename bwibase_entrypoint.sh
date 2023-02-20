#!/bin/bash

# while getopts u:g: flag
# do
#     case "${flag}" in
#         u) userid=${OPTARG};;
#         g) groupid=${OPTARG};;
#     esac
# done
# echo "Assigning host user id $userid to docker user."

# usermod -u $userid bwi-docker
# sleep 2
# groupmod -g $groupid bwi-docker

source /opt/ros/melodic/setup.bash
#source /home/bwi-docker/catkin_ws/devel/setup.bash
# export SEGWAY_INTERFACE_ADDRESS=10.66.171.1
# export SEGWAY_IP_ADDRESS=10.66.171.5
# export SEGWAY_IP_PORT_NUM=8080
# export SEGWAY_BASE_PLATFORM=RMP_110
# export SEGWAY_PLATFORM_NAME=RMP_110

# # ensure the postgresql database is accessible to the user
# sudo /etc/init.d/postgresql start
# sudo -u postgres psql -c "ALTER USER postgres WITH PASSWORD 'nopass'"
# echo -e '# hostname:port:database:username:password\n\
# localhost:*:knowledge_base:postgres:nopass\n' > ~/.pgpass
# sudo chmod 600 ~/.pgpass

# # build the knowlege_base db
# prepare_knowledge_bwi_ahg
# echo "Knowledge_base db setup complete"

#start ros
roscore