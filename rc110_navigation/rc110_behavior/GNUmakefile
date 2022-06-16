include ../../mk/common.mk

deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install --os ${ROS_OS} -iry --from-paths .

build:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	$(call build,rc110_behavior,,${cmake_flags})

run:
	source ${ws_path}/install/setup.bash
	ros2 launch rc110_behavior main.launch

show:
	$(MAKE) -C ../.. show
