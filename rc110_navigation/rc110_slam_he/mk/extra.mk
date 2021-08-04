map_name ?= map

save-map:
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosrun map_server map_saver -f ~/.ros/$(map_name)

select-map:
ifeq (,$(shell grep "RC110_MAP_NAME=" ~/.config/rc110/service.conf))
	echo "
	# Map name for SLAM and navigation
	RC110_MAP_NAME=$(map_name)
	"\
	>> ~/.config/rc110/service.conf
else
	sed -i 's/RC110_MAP_NAME=.*/RC110_MAP_NAME=$(map_name)/' ~/.config/rc110/service.conf
endif
