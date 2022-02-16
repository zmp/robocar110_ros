include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths . --skip-keys="rc110_msgs rc110_master_hold"

all:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin build --this --cmake-args ${cmake_flags}

run:
	$(call get_flags,flags)
	source $$(catkin locate --devel)/setup.bash
	roslaunch rc110_teleop teleop.launch $${flags}

mouse:
	$(call get_flags,flags)
	source $$(catkin locate --devel)/setup.bash
	roslaunch rc110_teleop mouse_teleop.launch $${flags}
