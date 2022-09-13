include ../../mk/common.mk

deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths . ../../rc110_core

build:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin build --this --cmake-args ${cmake_flags}

run:
	export RC110_HOST_PREFIX="-disabled-"  # no sync
	source $$(catkin locate rc110)/host_setup.bash
	mon launch rc110_gazebo simulation.launch $(ros_args)

show:
	source $$(catkin locate rc110)/host_setup.bash
	$(eval n:=$(or ${n},1))
	roslaunch --wait rc110_rviz multi.launch config:=$(shell pwd)/rviz/gazebo${n}.rviz
