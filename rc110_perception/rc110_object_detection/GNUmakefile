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
	source ../../../../devel/setup.bash
	source ~/.config/rc110/service.conf
	roslaunch rc110_object_detection rc110_object_detection_node.launch

show:
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosrun rviz rviz -d rviz/rc110_object_detection_node.rviz
	# without roslaunch, so it worked also on remote host without building the package

monitor:
	$(MAKE) -C ../.. env
	source ../../../../env.sh
	$(MAKE) show
