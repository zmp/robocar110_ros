include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths . --skip-keys="rc110_msgs rc110_topic_tools"

build:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin build --this --cmake-args ${cmake_flags}

run:
	source $$(catkin locate rc110)/auto_setup.bash
	$$(catkin locate --devel)/lib/rc110_rviz/rc110_rviz_app

show:
	source $$(catkin locate rc110)/auto_setup.bash
	roslaunch --wait rc110_rviz uni.launch

multi-show:
	source $$(catkin locate rc110)/host_setup.bash
	roslaunch --wait rc110_rviz multi.launch
