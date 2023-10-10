# Convenient script to use with Windows nmake.

default_target: rviz

rviz:
	cd ..\..
	c:\opt\ros\melodic\x64\setup.bat & \
	python3 ${yaml_parser} -f ${rviz_file} & \
	colcon build --only-pkg-with-deps rc110_rviz -DCMAKE_BUILD_TYPE=Release

show:
	python3 ${yaml_parser} -f ${rviz_file} & \
	..\..\devel\setup.bat & \
	ros2 launch rc110_rviz uni.launch

show-slam-cg:
	..\..\devel\setup.bat & \
	ros2 run rviz rviz -d rc110_navigation/rc110_slam_he/rviz/rc110_slam_cg.rviz

show-navigation:
	..\..\devel\setup.bat & \
	ros2 launch rviz rviz -d rc110_navigation/rc110_navigation/rviz/rc110_navigation.rviz

clean:
	cd ..\..
	rd /q /s build devel
