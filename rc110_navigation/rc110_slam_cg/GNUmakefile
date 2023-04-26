include ../../mk/common.mk

deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install --os ${ROS_OS} -iry  --from-paths .

build:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	$(call build,rc110_slam_cg,,${cmake_flags})

run:
	source ${ws_path}/install/setup.bash
	source /opt/ros/rolling/share/cartographer_ros/local_setup.bash
	source /opt/ros/rolling/share/cartographer_ros_msgs/local_setup.bash
	source ../../../imu_tools/install/setup.bash
	ros2 launch rc110_slam_cg main.launch $(ros_args)

show:
	source ${ws_path}/install/setup.bash
	source /opt/ros/rolling/share/cartographer_ros/local_setup.bash
	source /opt/ros/rolling/share/cartographer_ros_msgs/local_setup.bash
	source ../../../imu_tools/install/setup.bash
	ros2 launch rc110_rviz uni.launch config:=$$(pwd)/rviz/rc110_slam_cg.rviz
	
# extra
include mk/extra.mk
