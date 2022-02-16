include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths .

all:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin build --this --cmake-args ${cmake_flags}

run:
	$(call get_flags,flags)
	$(MAKE) -C ../.. init-run
	source $$(catkin locate --devel)/setup.bash
	source ~/.config/rc110/service.conf
	roslaunch rc110_slam_he main.launch $${flags}

show:
	source $$(catkin locate --devel)/setup.bash
	rosrun rviz rviz -d rviz/rc110_slam_he.rviz
	# without roslaunch, so it worked also on remote host without building the package

monitor:
	$(MAKE) -C ../.. env
	source $$(catkin locate)/env.sh
	$(MAKE) show

# extra
include mk/extra.mk
