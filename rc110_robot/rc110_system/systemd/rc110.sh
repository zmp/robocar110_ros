#!/bin/bash

source /usr/lib/systemd/user/rc110-prepare.sh

config_file=~/.config/rc110/service.conf

# Create config file if not exists.
if [ ! -f $config_file ]; then
    mkdir -p ~/.config/rc110
    cp /opt/ros/@ROS_DISTRO@/share/rc110_system/service_template.conf $config_file
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
