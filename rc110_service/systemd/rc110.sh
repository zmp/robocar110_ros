#!/bin/bash

source /opt/ros/@ROS_DISTRO@/setup.bash

# Give ifconfig 10 seconds to get the ip address.
for i in {1..10}; do
    export ROS_IP=$(ifconfig "${RC110_INTERFACE}" | grep 'inet ' | awk '{print $2}')

    if [ -n "$ROS_IP" ]; then
        break;
    fi
    sleep 1
done

if [ -z "$ROS_IP" ]; then
    echo "Failed command: ifconfig ${RC110_INTERFACE}"
    exit 1
fi

roslaunch --wait rc110_launch robot.launch
