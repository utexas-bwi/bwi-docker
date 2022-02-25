#!/bin/bash
# entrypoint

#source the ros setup script
source /opt/ros/melodic/setup.bash

# set env variables
export SEGWAY_INTERFACE_ADDRESS=10.66.171.1
export SEGWAY_IP_ADDRESS=10.66.171.5
export SEGWAY_IP_PORT_NUM=8080
export SEGWAY_BASE_PLATFORM=RMP_110
export SEGWAY_PLATFORM_NAME=RMP_110

#start ros
roscore

# Execute the command passed into this entrypoint
exec "$@"