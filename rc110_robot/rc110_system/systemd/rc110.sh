#!/bin/bash

source /opt/ros/@ROS_DISTRO@/rc_setup.bash

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
