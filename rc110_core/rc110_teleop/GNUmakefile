include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths . ../rc110_msgs

build:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin build --this --cmake-args ${cmake_flags}

run:
	source $$(catkin locate rc110)/host_setup.bash
	roslaunch --wait rc110_teleop teleop.launch $(ros_args)

mouse:
	source $$(catkin locate rc110)/auto_setup.bash
	roslaunch --wait rc110_teleop mouse_teleop.launch $(ros_args)
