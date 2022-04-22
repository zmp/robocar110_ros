include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths . ../../rc110_core

build:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin build --this --cmake-args ${cmake_flags}

run:
	export RC110_PREFIX="grc_"  # exclude this prefix from sync
	source $$(catkin locate rc110)/host_setup.bash
	mon launch rc110_gazebo simulation.launch $(ros_args)

show:
	source $$(catkin locate rc110)/host_setup.bash
	$(eval ng0:=$(filter 1,$(shell expr "${n}" '>' 0)))
	RC110_RVIZ_CONFIG=$(if ${ng0},"$(shell pwd)/rviz/gazebo${n}.rviz","$(shell catkin locate rc110_rviz)/rviz/main.rviz")
	roslaunch --wait rc110_rviz rviz.launch config:=$${RC110_RVIZ_CONFIG}
