#!/bin/bash

source /usr/lib/systemd/user/rc110-prepare.sh

config_file=~/.config/rc110/service.conf

# Create config file if not exists.
if [ ! -f $config_file ]; then
    mkdir -p ~/.config/rc110
    echo "
# RC110 service configuration file

# Uncomment and specify IP of this robot to connect remotely.
# We recommend to specify the same static IP in Network Manager.
#     Other way: you can resolve hostname (zmp) to IP on client PC (e.g. in hosts file).
#ROS_IP=192.168.110.5

# Change to your launch command, if you created another one.
RC110_LAUNCH_COMMAND='roslaunch --wait rc110_launch robot.launch use_front_camera:=true joy_type:=elecom'

# Map name for SLAM and navigation
RC110_MAP_NAME=map
"\
        > $config_file
fi

# Add config to bashrc too.
if ! grep -q "Export RC110" ~/.bashrc; then
    echo "
# Export RC110 variables from the config file
set -a
. $config_file
set +a
"\
        >> ~/.bashrc
fi

# Apply config
set -a
. $config_file
set +a

echo "ROS_IP=$ROS_IP"

# Disable network, if IP address is not available during 30s. If not disabled, ROS will fail to start.
if [ -n "$ROS_IP" ]; then
    echo "Waiting for $ROS_IP interface (up to 30s)..."

    is_ip_assigned="false"
    for i in {1..30}; do
        if hostname -I | tr ' ' '\n' | grep -q $ROS_IP; then
            is_ip_assigned="true"
            break;
        fi
        sleep 1
    done

    if [ "$is_ip_assigned" == "false" ]; then
        echo "Unable to find ip address $ROS_IP, so ROS_IP is disabled."
        unset ROS_IP
    else
        echo "Network interface found."
    fi
fi

# Launch ros nodes
eval "$RC110_LAUNCH_COMMAND"
