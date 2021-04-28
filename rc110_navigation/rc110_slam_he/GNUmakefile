include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF

init:
	cd ../.. && $(MAKE) init

deps: init
	$(call source)
	rosdep install -iry --from-paths . --skip-keys=rc110_laserscans_to_pointcloud

all: init
	$(call source)
	catkin build --this --cmake-args ${cmake_flags}

run:
	$(call get_flags,flags)
	source ../../../../devel/setup.bash
	roslaunch rc110_slam_he main.launch $${flags}

show:
	source ../../../../devel/setup.bash
	rosrun rviz rviz -d rviz/rc110_slam_he.rviz
	# without roslaunch, so it worked also on remote host without building the package

monitor:
	$(MAKE) env -C ../..
	source ../../../../env.sh
	$(MAKE) show
