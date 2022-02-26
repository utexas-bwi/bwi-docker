#!/bin/bash

# source workspaces
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source /home/bwilab/catkin_ws/devel/setup.bash" >> ~/.bashrc

# set env variables
echo "
export SEGWAY_INTERFACE_ADDRESS=10.66.171.1
export SEGWAY_IP_ADDRESS=10.66.171.5
export SEGWAY_IP_PORT_NUM=8080
export SEGWAY_BASE_PLATFORM=RMP_110
export SEGWAY_PLATFORM_NAME=RMP_110
" >> ~/.bashrc

source /opt/ros/melodic/setup.bash
source /home/bwilab/catkin_ws/devel/setup.bash

# ensure the postgresql database is accessible to the user
sudo service postgresql start
# sleep 1
sudo -u postgres psql -c "ALTER USER postgres WITH PASSWORD 'nopass'"
cat > ~/.pgpass <<EOF
# hostname:port:database:username:password
localhost:*:knowledge_base:postgres:nopass
EOF
sudo chmod 600 ~/.pgpass
prepare_knowledge_bwi_ahg

#start ros
roscore

# Execute the command passed into this entrypoint
exec "$@"