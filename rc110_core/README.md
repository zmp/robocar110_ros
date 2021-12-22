# RoboCar 1/10 Core Nodes

It's a set of major ROS nodes for Robocar 1/10.

They are preinstalled on robot and run with systemd service.

This node group tends to have more support than others.

| Name                                               | Description             |
|:------------------                                 |:------------------------|
| [**drive_control**](rc110_drive_control/README.md) | robot ros driver        |
| [**launch**](rc110_launch/README.md)               | default launch files    |
| **msgs**                                           | robot specific messages |
| **rviz**                                           | rviz control panel      |
| [**system**](rc110_system/README.md)               | systemd service package |
| [**teleop**](rc110_teleop/README.md)               | joystick manipulation   |

## Build

```
cd ~/ros/src/robocar110_ros

make deps    # ros dependencies
make         # build core packages
```

## Package
Create deb files in the build directory:
```
make package
```

## Install
Install those files with `sudo apt-get install`:
```
make install
```

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

## ROS configuration

Default configuration is installed during packages installation.

If on update the configuration was changed in both new package and your local file, you will be prompted to adjust it manually.

* `/opt/ros/${ROS_DISTRO}/share/rc110_launch/config/`
