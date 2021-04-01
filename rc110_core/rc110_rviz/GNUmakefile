include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


init:
	cd ../.. && $(MAKE) init

deps: init
	$(call source)
	rosdep install -iry --from-paths . --skip-keys=rc110_msgs

all: init
	$(call source)
	catkin build --this --cmake-args ${cmake_flags}

run:
	source ../../../../devel/setup.bash
	$$(catkin locate --devel)/lib/rc110_rviz/rc110_rviz_app

show:
	source ../../../../devel/setup.bash
	roslaunch rc110_rviz rviz.launch

monitor:
	$(MAKE) env -C ../..
	source ../../../../env.sh
	$(MAKE) show