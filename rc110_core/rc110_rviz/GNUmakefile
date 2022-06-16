include ../../mk/common.mk

deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --os ${ROS_OS} --from-paths . ../rc110_msgs ../rc110_topic_tools ../rviz_video_panel

build:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	$(call build,rc110_rviz,,${cmake_flags})

show:
	source ${ws_path}/install/setup.bash
	ros2 launch rc110_rviz uni.launch $(ros_args)

multi-show:
	source ${ws_path}/install/setup.bash
	ros2 launch rc110_rviz multi.launch $(ros_args)
