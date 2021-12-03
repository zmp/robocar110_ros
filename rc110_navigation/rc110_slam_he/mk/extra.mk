map_name ?= map

save-map-he:
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosrun map_server map_saver -f ~/.ros/$(map_name)
