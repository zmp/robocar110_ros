#!/bin/bash

source /usr/lib/systemd/user/rc110-prepare.sh
source /opt/ros/@ROS_DISTRO@/share/rc110/env/config.bash

# Disable ROS_IP in favor of avahi
unset ROS_IP

# Add config to bashrc too.
if ! grep -q "Export RC110" ~/.bashrc; then
    echo "
# Export RC110 variables from the config file
set -a
source $config_file
set +a
"\
        >> ~/.bashrc
fi

# Launch ros nodes
eval "$RC110_LAUNCH_COMMAND"
