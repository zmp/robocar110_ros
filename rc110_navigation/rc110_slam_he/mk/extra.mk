map_name ?= map

save-map-he:
	source $$(catkin locate rc110)/auto_setup.bash
	rosrun map_server map_saver -f ~/.ros/$(map_name)
