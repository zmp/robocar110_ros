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
	roslaunch rc110_slam_cg main.launch

show:
	source ../../../../devel/setup.bash
	rosrun rviz rviz -d rviz/rc110_slam_cg.rviz
	# without roslaunch, so it worked also on remote host without building the package

monitor:
	$(MAKE) env -C ../..
	source ../../../../env.sh
	$(MAKE) show