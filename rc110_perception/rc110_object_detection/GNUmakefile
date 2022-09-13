include ../../mk/common.mk

deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths . ../../rc110_core/rc110_msgs

build:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin build --this --cmake-args ${cmake_flags}

run:
	source $$(catkin locate rc110)/auto_setup.bash
	roslaunch --wait rc110_object_detection rc110_object_detection_node.launch

show:
	source $$(catkin locate rc110)/auto_setup.bash
	roslaunch --wait rc110_rviz uni.launch config:=$$(pwd)/rviz/rc110_object_detection_node.rviz
