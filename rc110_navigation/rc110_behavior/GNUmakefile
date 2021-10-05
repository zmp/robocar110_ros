include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths .

all:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin build --this --cmake-args ${cmake_flags}

run:
	$(MAKE) -C ../.. init-run
	source ../../../../devel/setup.bash
	source ~/.config/rc110/service.conf
	mon launch rc110_behavior main.launch

show:
	$(MAKE) -C ../.. show

monitor:
	$(MAKE) -C ../.. monitor
