include ../../mk/common.mk

deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --os ${ROS_OS} --from-paths . ../../rc110_core/rc110_msgs

build:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	$(call build,rc110_object_detection,,${cmake_flags})

run:
	source ${ws_path}/install/setup.bash
	ros2 launch rc110_object_detection rc110_object_detection_node.launch

show:
	source ${ws_path}/install/setup.bash
	ros2 launch rc110_rviz uni.launch config:=$$(pwd)/rviz/rc110_object_detection_node.rviz
