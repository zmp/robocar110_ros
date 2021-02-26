# RoboCar 1/10 ROS nodes

It's a set of default ROS nodes for Robocar 1/10.

They are preinstalled on robot and run with systemd service.

| Name                                               | Description             |
|:------------------                                 |:------------------------|
| [**behavior**](rc110_behavior/README.md)           | simple AD example       |
| [**drive_control**](rc110_drive_control/README.md) | robot ros driver        |
| [**launch**](rc110_launch/README.md)               | default launch files    |
| **msgs**                                           | robot specific messages |
| **rviz**                                           | rviz control panel      |
| [**service**](rc110_service/README.md)             | systemd service package |
| [**teleop**](rc110_teleop/README.md)               | joystick manipulation   |

By default, most of the nodes run with 30 Hz frequency.

## Prerequisites
#### ROS

Please, install **ROS melodic** on **Ubuntu** following the instruction:

http://wiki.ros.org/Installation/Ubuntu

#### RC110 Source

```
export ROS_DISTRO=melodic
source /opt/ros/${ROS_DISTRO}/setup.bash

mkdir -p ~/ros/src
cd ~/ros/src
git clone <url or this repo>  # or unzip from archive

cd ~/ros/src/robocar110_ros/
```

#### Init Catkin and RosDep

```
make init
```

#### ROS Environment

```
make ros-source
```
* It will place ros sourcing to `~/.bashrc`.
* You need to restart terminal after that.

## Build

```
make deps    # ros dependencies

make
```
* It will build all with catkin build.

## Package

```
make package
```
* It will create deb files in build directory.

## Install

```
make install
```
* It will install those files with `sudo apt-get install`.

## RViz Panel

```
make deps-rviz
make rviz
```
* It will build rviz plugin in the current ros workspace.

```
make show
```
* to show RViz with default layout

## ROS configuration

Configuration is installed during packages installation.

It is possible to edit the configuration, the next package update won't change it.

* `/opt/ros/${ROS_DISTRO}/share/rc110/config/`

## ~ Additional ~

* [**Windows**](docs/Windows.md)
* [**Advanced Nodes**](advanced/README.md)
* [**Contribution Rules**](docs/Contribution.md)