# Convenient script to use with Windows nmake.

default_target: rviz

rviz:
	cd ..\..
	c:\opt\ros\melodic\x64\setup.bat & \
	catkin_make --only-pkg-with-deps rc110_rviz -DCMAKE_BUILD_TYPE=Release

show:
!IF EXIST(..\..\env.bat)
	..\..\env.bat & \
!ELSE
!MESSAGE Create ..\..\env.bat with similar contents:
!MESSAGE
!MESSAGE set ROS_MASTER_URI=http://192.168.110.5:11311
!MESSAGE set ROS_IP=192.168.110.3
!MESSAGE
!ERROR
!ENDIF
	..\..\devel\setup.bat & \
	roslaunch rc110_rviz rviz.launch

clean:
	cd ..\..
	rd /q /s build devel
