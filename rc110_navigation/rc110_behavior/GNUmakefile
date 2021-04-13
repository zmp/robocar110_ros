include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


init:
	cd ../.. && $(MAKE) init

deps: init
	$(call source)
	rosdep install -iry --from-paths .

all: init
	$(call source)
	catkin build --this --cmake-args ${cmake_flags}

run:
	source ../../../../devel/setup.bash
	mon launch rc110_behavior main.launch

show:
	cd ../.. && $(MAKE) show

monitor:
	cd ../.. && $(MAKE) monitor
