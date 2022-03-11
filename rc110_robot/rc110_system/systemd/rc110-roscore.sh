#!/bin/bash

source /usr/lib/systemd/user/rc110-prepare.sh

echo "Starting roscore with ROS_HOSTNAME=${ROS_HOSTNAME}"

roslaunch rc110 multimaster.launch
