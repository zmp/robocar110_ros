.DEFAULT_GOAL := all
.ONESHELL:
.SHELLFLAGS := -ec
SHELL := /bin/bash

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF

define source
	source /opt/ros/melodic/setup.bash
endef

init:
	cd ../.. && $(MAKE) init

deps: init
	$(call source)
	rosdep install -iry --from-paths .

all: init
	$(call source)
	catkin build --this --cmake-args ${cmake_flags}

run:
	source ../../../../devel/setup.bash
	roslaunch --screen rc110_video_server main.launch

show:
	if [ -f ../../../../env.sh ]; then
		source ../../../../env.sh
		# On remote client, please, fill the file similar to:
		#
		# export ROS_MASTER_URI=http://192.168.110.5:11311
		# export ROS_IP=192.168.110.2
	fi
	if [[ $$ROS_MASTER_URI =~ http://(.*):[0-9]+ ]]; then
		ip=$${BASH_REMATCH[1]}
		gst-launch-1.0 rtspsrc latency=0 location=rtsp://$$ip:8554/front \
			! rtph265depay ! h265parse ! queue ! avdec_h265 ! fpsdisplaysink sync=false async=false
	else
		echo "Unable to parse ROS_MASTER_URI"
	fi
	# ------------------------------------------------------------------------------
