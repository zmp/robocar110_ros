include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths . --skip-keys=rc110_twist_to_ackermann

build:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin build --this --cmake-args ${cmake_flags}

run:
	source $$(catkin locate rc110)/env/devel.bash
	source $$(catkin locate rc110)/env/config.bash
	mon launch rc110_navigation main.launch

show:
	source $$(catkin locate rc110)/env/devel.bash
	rosrun rc110 launch rc110_rviz rviz.launch config:=$$(pwd)/rviz/rc110_navigation.rviz

# extra
include mk/extra.mk
