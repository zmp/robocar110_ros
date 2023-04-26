run-model:
	source /opt/ros/${ROS_DISTRO}/setup.bash
	ros2 launch --wait rc110_gazebo model.launch $(ros_args)

run-navigation-zmp:
	source ${ws_path}/install/setup.bash
	ros2 launch rc110_navigation main.launch map_path:=$$(pwd)/maps/zmp.yaml $(ros_args)