#!/bin/bash

while getopts w: flag
do
    case "${flag}" in
        w) ws=${OPTARG};;
    esac
done

# source the given workspace after /opt/ros...
if [ -n "$ws" ]; then
sed -i '/source \/opt\/ros\/melodic\/setup.bash/a \
source '$ws'\/devel\/setup.bash' /home/bwi-docker/.profile
sed -i '/source \/opt\/ros\/melodic\/setup.bash/a \
source '$ws'\/devel\/setup.bash' /home/bwi-docker/.bashrc
fi

source /opt/ros/melodic/setup.bash

# ensure the postgresql database is accessible to the user
sudo /etc/init.d/postgresql start
sudo -u postgres psql -c "ALTER USER postgres WITH PASSWORD 'nopass'"
echo -e '# hostname:port:database:username:password\n\
localhost:*:knowledge_base:postgres:nopass\n' > ~/.pgpass
sudo chmod 600 ~/.pgpass

# build the knowlege_base db
# prepare_knowledge_bwi_ahg &&\
# echo "Knowledge_base db setup complete"

#start ros
roscore