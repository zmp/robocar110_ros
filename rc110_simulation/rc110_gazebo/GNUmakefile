include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


init:
	cd ../.. && $(MAKE) init

deps: init
	$(call source)
	rosdep install -iry --from-paths . --skip-keys="rc110_msgs rc110_common"

all: init
	$(call source)
	catkin build --this --cmake-args ${cmake_flags}

run:
	$(call get_flags,flags)
	source ../../../../devel/setup.bash
	export GAZEBO_MODEL_DATABASE_URI=" "   # See: https://github.com/osrf/gazebo/issues/2934
	roscore &>/dev/null &
	mon launch rc110_gazebo main.launch $${flags}

show:
	cd ../.. && $(MAKE) show
