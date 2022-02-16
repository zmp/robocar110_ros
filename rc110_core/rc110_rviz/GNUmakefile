include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths . --skip-keys=rc110_msgs

all:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin build --this --cmake-args ${cmake_flags}

run:
	source $$(catkin locate --devel)/setup.bash
	$$(catkin locate --devel)/lib/rc110_rviz/rc110_rviz_app

show:
	source $$(catkin locate --devel)/setup.bash
	roslaunch rc110_rviz rviz.launch

monitor:
	$(MAKE) -C ../.. env
	source $$(catkin locate)/env.sh
	$(MAKE) show