#!/bin/bash

source /opt/ros/@ROS_DISTRO@/setup.bash

export ROS_IP=$(ifconfig "${RC110_INTERFACE}" | grep 'inet ' | awk '{print $2}')
roscore
