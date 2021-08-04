include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


init:
	cd ../.. && $(MAKE) init

deps: init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths .

all: init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin build --this --cmake-args ${cmake_flags}

run:
	source ../../../../devel/setup.bash
	source ~/.config/rc110/service.conf
	mon launch rc110_behavior main.launch

show:
	cd ../.. && $(MAKE) show

monitor:
	cd ../.. && $(MAKE) monitor
