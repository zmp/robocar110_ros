# RoboCar 1/10 Core Nodes

It's a set of ROS nodes common for robot and simulation.

| Name                                                                     | Description                                             |
|:-------------------------------------------------------------------------|:--------------------------------------------------------|
| [**common**](rc110_common/README.md)                                     | Common launch files                                     |
| [**laserscans_to_pointcloud**](rc110_laserscans_to_pointcloud/README.md) | Conversion from two LaserScan to PointCloud2            |
| [**master_hold**](rc110_master_hold/README.md)                           | Node that does reconnection to roscore when it restarts |
| **msgs**                                                                 | Robot specific messages                                 |
| **rviz**                                                                 | RViz control panel                                      |
| [**teleop**](rc110_teleop/README.md)                                     | Joystick manipulation                                   |
| [**twist_to_ackermann**](rc110_twist_to_ackermann/README.md)             | Conversion from special Twist to Ackermann              |


## Remote Configuration

In order to connect to robot from remote PC, `ROS_MASTER_URI` and `ROS_IP` environment variables should be set. `env.sh` file is created automatically for this purpose to use with `make` command.

If you want to configure those variables beforehand, please, run the following command and adjust the variables.
```
make env
```

## RViz Panel
```
make deps-rviz
make rviz        # Build rviz plugin in the current ros workspace
make show        # Show RViz with default layout locally
make monitor     # Show RViz with default layout on remote PC
```

## Joystick Connected to PC
Connect joystick dongle to your remote PC instead of robot:
```
make deps-teleop
make teleop
make remote-teleop
```

Connect other joystick to remote PC:
```
make remote-teleop device:=js1 joy_type:=logicool
```

* To create new joystick configuration in zmp repository, see example configurations in `rc110_core/rc110_common/config/`

## Mouse Operation
If you want to move the robocar by mouse operation, do the following:  
* Build package:  
```
make deps-teleop
make teleop
```  

* Launch:  
```
cd ~/ros
source devel/setup.bash
roslaunch rc110_teleop mouse_teleop.launch
```

## ROS configuration

Default configuration is installed during packages installation.

If on update the configuration was changed in both new package and your local file, you will be prompted to adjust it manually.

* `/opt/ros/${ROS_DISTRO}/share/rc110_launch/config/`
