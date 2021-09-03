include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths . --skip-keys="rc110_msgs rc110_common"

all:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin build --this --cmake-args ${cmake_flags}

run:
	$(call get_flags,flags)
	source ../../../../devel/setup.bash
	export GAZEBO_MODEL_DATABASE_URI=" "   # See: https://github.com/osrf/gazebo/issues/2934
	roscore &>/dev/null &
	mon launch rc110_gazebo main.launch $${flags}
	kill $$!

show:
	$(MAKE) -C ../.. show
