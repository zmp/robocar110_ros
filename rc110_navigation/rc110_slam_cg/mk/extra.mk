map_name ?= map
map_resolution ?= 0.025

save-map-cg:
	source ${ws_path}/install/setup.bash
	ros2 service call $${ROS_NAMESPACE}/write_state cartographer.pbstream true
	cd ~/.ros
	ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
		-pbstream_filename cartographer.pbstream \
		-map_filestem ${map_name} \
		-resolution ${map_resolution}