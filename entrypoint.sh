#!/bin/bash

set -e

echo "in the entrypoint script"
while getopts w: flag
do
    case "${flag}" in
        w) ws=${OPTARG};;
    esac
done

# source the given workspace after /opt/ros...
if [ -n "$ws" ]; then
echo "updating ~/.profile and ~/.bashrc with workspace path"
sed -i '/source \/opt\/ros\/melodic\/setup.bash/a \
source '$ws'\/devel\/setup.bash' /home/bwi-docker/.profile
sed -i '/source \/opt\/ros\/melodic\/setup.bash/a \
source '$ws'\/devel\/setup.bash' /home/bwi-docker/.bashrc
fi

cat > ~/.pgpass <<EOF
# hostname:port:database:username:password
localhost:*:knowledge_base:postgres:nopass
EOF
chmod 600 ~/.pgpass

source /opt/ros/melodic/setup.bash

#start ros
roscore