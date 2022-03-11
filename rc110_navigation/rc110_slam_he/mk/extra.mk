map_name ?= map

save-map-he:
	source $$(catkin locate rc110)/env/devel.bash
	rosrun map_server map_saver -f ~/.ros/$(map_name)
