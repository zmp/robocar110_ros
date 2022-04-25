#!/bin/bash

source /opt/ros/@ROS_DISTRO@/rc_setup.bash on_robot

echo "Starting roscore with ROS_HOSTNAME=${ROS_HOSTNAME}"

roslaunch rc110 multimaster.launch
