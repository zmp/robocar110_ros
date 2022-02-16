include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths . --skip-keys=rc110_twist_to_ackermann

all:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin build --this --cmake-args ${cmake_flags}

run:
	$(MAKE) -C ../.. init-run
	source $$(catkin locate --devel)/setup.bash
	source ~/.config/rc110/service.conf
	export RC110_MAP_NAME=$${RC110_MAP_NAME:-map}
	mon launch rc110_navigation main.launch

show:
	source $$(catkin locate --devel)/setup.bash
	rosrun rviz rviz -d rviz/rc110_navigation.rviz

monitor:
	$(MAKE) -C ../.. env
	source $$(catkin locate)/env.sh
	$(MAKE) show

# extra
include mk/extra.mk
