# Convenient script to use with Windows nmake.

default_target: rviz

rviz:
	cd ..\..
	c:\opt\ros\melodic\x64\setup.bat & \
	catkin_make --only-pkg-with-deps rc110_rviz -DCMAKE_BUILD_TYPE=Release

show:
	..\..\devel\setup.bat & \
	roslaunch rc110_rviz rviz.launch

show-slam-he:
	..\..\devel\setup.bat & \
	rosrun rviz rviz -d rc110_navigation/rc110_slam_he/rviz/rc110_slam_he.rviz

show-navigation:
	..\..\devel\setup.bat & \
	rosrun rviz rviz -d rc110_navigation/rc110_navigation/rviz/rc110_navigation.rviz

clean:
	cd ..\..
	rd /q /s build devel
