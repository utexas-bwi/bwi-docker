#!/bin/bash

set -e

echo "in the entrypoint script"
while getopts :w:u:n: flag; do
    case "${flag}" in
        w)
          ws=${OPTARG}
          ;;
        u)
          userid=${OPTARG}
          ;;
        n)
          uname=${OPTARG}
          ;;
    esac
done

# V4 Env variables have to be sourced after the bwi workspaces
echo "updating ~/.profile and ~/.bashrc with V4 Env variables"
sed -i '/source \/opt\/ros\/melodic\/setup.bash/a \
source ~/base_env/v4_env\n' /root/.profile
sed -i '/source \/opt\/ros\/melodic\/setup.bash/a \
source ~/base_env/v4_env\n' /root/.bashrc

# adding V4 convenience function
echo -e "sv4 () {\n\
source ~/base_env/v4_env\n\
}\n" >> ~/.profile && \
echo -e "sv4 () {\n\
source ~/base_env/v4_env\n\
}\n" >> ~/.bashrc

# source the given workspace after /opt/ros...
if [ -n "$ws" ]; then
echo "updating ~/.profile and ~/.bashrc with workspace path"
sed -i '/source \/opt\/ros\/melodic\/setup.bash/a \
source '$ws'\/devel\/setup.bash' /root/.profile
sed -i '/source \/opt\/ros\/melodic\/setup.bash/a \
source '$ws'\/devel\/setup.bash' /root/.bashrc
fi

# get host user and ensure they own the projects dir
echo "setting projects volume owner to $uname $userid"
export uid=${userid%"${userid#????}"}
useradd -u $uid -s /bin/bash -G dialout $uname
chown -R $userid /root/projects

cat > ~/.pgpass <<EOF
# hostname:port:database:username:password
localhost:*:knowledge_base:postgres:nopass
EOF
chmod 600 ~/.pgpass

source /opt/ros/melodic/setup.bash

#start ros
roscore