map_name ?= map
map_resolution ?= 0.025

save-map-cg:
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosservice call /write_state cartographer.pbstream
	cd ~/.ros
	cartographer_pbstream_to_ros_map \
		-pbstream_filename cartographer.pbstream \
		-map_filestem ${map_name} \
		-resolution ${map_resolution}
