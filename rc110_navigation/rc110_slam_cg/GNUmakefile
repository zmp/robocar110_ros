include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


init:
	cd ../.. && $(MAKE) init

deps: init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths .

all: init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin build --this --cmake-args ${cmake_flags}

run:
	source ../../../../devel/setup.bash
	source ~/.config/rc110/service.conf
	roslaunch rc110_slam_cg main.launch

show:
	source ../../../../devel/setup.bash
	rosrun rviz rviz -d rviz/rc110_slam_cg.rviz
	# without roslaunch, so it worked also on remote host without building the package

monitor:
	$(MAKE) env -C ../..
	source ../../../../env.sh
	$(MAKE) show