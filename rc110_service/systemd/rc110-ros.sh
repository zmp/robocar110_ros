#!/bin/bash

source /opt/ros/@ROS_DISTRO@/setup.bash
export ROS_IP=$(hostname -I | cut -d' ' -f1)
roscore
