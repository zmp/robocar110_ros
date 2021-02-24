.DEFAULT_GOAL := all
.ONESHELL:
.SHELLFLAGS := -ec
SHELL := /bin/bash

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF

define source
	source /opt/ros/melodic/setup.bash
endef

deps:
	$(call source)
	rosdep install -iry --from-paths .

all:
	$(call source)
	catkin build --this --cmake-args ${cmake_flags}

run:
	source ../../../../devel/setup.bash
	roslaunch rc110_slam_cg main.launch

show:
	if [ -f ../../../../env.sh ]; then
		source ../../../../env.sh
		# On remote client, please, fill the file similar to:
		#
		# export ROS_MASTER_URI=http://192.168.110.5:11311
		# export ROS_IP=192.168.110.2
	fi
	source ../../../../devel/setup.bash
	roslaunch rc110_slam_cg rviz.launch
