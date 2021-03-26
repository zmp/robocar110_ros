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
	roslaunch rc110_object_detection rc110_object_detection_node.launch

show:
	source ../../../../devel/setup.bash
	roslaunch launch/rviz.launch  # by path, because no need to build on remote host

remote-show:
	$(MAKE) env -C ../..
	source ../../../../env.sh
	$(MAKE) show
