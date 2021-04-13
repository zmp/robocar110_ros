map_name ?= map

save-map:
	$(call source)
	rosrun map_server map_saver -f ~/.ros/$(map_name)

select-map:
	echo "map_name=$(map_name)" > ~/.ros/map.sh
