#!/bin/bash

source /opt/ros/@ROS_DISTRO@/setup.bash
source /home/zmp/ros/devel/setup.bash

roslaunch --wait rc110_launch robot.launch
