# Convenient script to use with Windows nmake.

default_target: rviz

rviz:
	cd ..\..
	c:\opt\ros\melodic\x64\setup.bat & \
	catkin_make --only-pkg-with-deps rc110_rviz -DCMAKE_BUILD_TYPE=Release

env:
!IF !EXIST(..\..\env.bat)
	copy mk\env_template.bat ..\..\env.bat
!ENDIF

monitor: env
	..\..\env.bat & \
	..\..\devel\setup.bat & \
	roslaunch rc110_rviz rviz.launch

monitor-slam-he: env
	..\..\env.bat & \
	..\..\devel\setup.bat & \
	rosrun rviz rviz -d rc110_navigation/rc110_slam_he/rviz/rc110_slam_he.rviz

monitor-navigation: env
	..\..\env.bat & \
	..\..\devel\setup.bat & \
	rosrun rviz rviz -d rc110_navigation/rc110_navigation/rviz/rc110_navigation.rviz

clean:
	cd ..\..
	rd /q /s build devel
