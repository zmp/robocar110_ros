include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


init:
	cd ../.. && $(MAKE) init

deps: init
	$(call source)
	rosdep install -iry --from-paths . --skip-keys=rc110_twist_to_ackermann

all: init
	$(call source)
	catkin build --this --cmake-args ${cmake_flags}

run:
	source ../../../../devel/setup.bash
ifneq (,$(wildcard ~/.ros/map.sh))
	source ~/.ros/map.sh
	mon launch rc110_navigation main.launch map_name:=$${map_name}
else
	mon launch rc110_navigation main.launch
endif

show:
	source ../../../../devel/setup.bash
	rosrun rviz rviz -d rviz/rc110_navigation.rviz

monitor:
	$(MAKE) env -C ../..
	source ../../../../env.sh
	$(MAKE) show

# extra
include mk/extra.mk
