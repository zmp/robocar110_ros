include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths . ../../rc110_core

build:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin build --this --cmake-args ${cmake_flags}

run:
	source $$(catkin locate rc110)/env/devel.bash
	source $$(catkin locate rc110)/env/config.bash
	export GAZEBO_MODEL_DATABASE_URI=" "   # See: https://github.com/osrf/gazebo/issues/2934
	rosrun rc110 mon rc110_gazebo simulation.launch $${RC110_ARGS} $(ros_args)

show:
	$(MAKE) -C ../.. show
