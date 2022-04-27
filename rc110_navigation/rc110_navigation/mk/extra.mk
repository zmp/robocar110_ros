map_name ?= map

define map_config_text

# Map name for SLAM and navigation
RC110_MAP_NAME=$(map_name)

endef

select-map:
ifeq (,$(shell grep "RC110_MAP_NAME=" ~/.config/rc110/config.bash 2>/dev/null))
	echo -e "${map_config_text}" >> ~/.config/rc110/config.bash
else
	sed -i 's/RC110_MAP_NAME=.*/RC110_MAP_NAME=$(map_name)/' ~/.config/rc110/config.bash
endif
