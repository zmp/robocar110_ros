include ../../mk/common.mk

deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install --os ${ROS_OS} -iry --from-paths . ../rc110_msgs

build:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	$(call build,rc110_teleop,,${cmake_flags})

run:
	source ${ws_path}/install/setup.bash
	ros2 launch rc110_teleop teleop.launch $(ros_args)

mouse:
	source ${ws_path}/install/setup.bash
	ros2 launch rc110_teleop mouse_teleop.launch $(ros_args)
