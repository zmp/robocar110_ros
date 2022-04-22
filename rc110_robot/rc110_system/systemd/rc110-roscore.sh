#!/bin/bash

source /opt/ros/@ROS_DISTRO@/rc_setup.bash

echo "Starting roscore with ROS_HOSTNAME=${ROS_HOSTNAME}"

export RC110_ROBOT=true
roslaunch rc110 multimaster.launch
