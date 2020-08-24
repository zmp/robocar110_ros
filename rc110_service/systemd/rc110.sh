#!/bin/bash

source /opt/ros/@ROS_DISTRO@/setup.bash
source /home/zmp/ros/devel/setup.bash
export ROS_IP=$(hostname -I | cut -d' ' -f1)

roslaunch --wait rc110_launch main.launch
