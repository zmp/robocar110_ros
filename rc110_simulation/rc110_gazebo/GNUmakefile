include ../../mk/common.mk

deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths . ../../rc110_core

build:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	$(call build,rc110_gazebo,,${cmake_flags})

run:
	export RC110_HOST_PREFIX="-disabled-"  # no sync
	source ${ws_path}/install/setup.bash
	source /usr/share/gazebo-11/setup.bash
	ros2 launch rc110_gazebo simulation.launch $(ros_args)

show:
	source ${ws_path}/install/setup.bash
	$(eval n:=$(or ${n},1))
	ros2 launch rc110_rviz multi.launch config:=$(shell pwd)/rviz/gazebo${n}.rviz