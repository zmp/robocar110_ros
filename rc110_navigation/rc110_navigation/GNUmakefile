include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths . --skip-keys="rc110_twist_to_ackermann goal_queue"

build:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin build --this --cmake-args ${cmake_flags}

run:
	source $$(catkin locate rc110)/auto_setup.bash
	export RC110_MAP_NAME
	mon launch rc110_navigation main.launch $(ros_args)

show:
	source $$(catkin locate rc110)/auto_setup.bash
	roslaunch --wait rc110_rviz uni.launch config:=$$(pwd)/rviz/rc110_navigation.rviz

# extra
include mk/extra.mk
