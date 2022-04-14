include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths .

build:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin build --this --cmake-args ${cmake_flags}

run:
	source $$(catkin locate rc110)/auto_setup.bash
	roslaunch --wait rc110_slam_cg main.launch

show:
	source $$(catkin locate rc110)/auto_setup.bash
	roslaunch --wait rc110_rviz rviz.launch config:=$$(pwd)/rviz/rc110_slam_cg.rviz

# extra
include mk/extra.mk
