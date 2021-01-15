# RoboCar 1/10 ROS nodes

It's a set of default ROS nodes for Robocar 1/10.

They are preinstalled on robot and run with systemd service.

| Name              | Description             |
|:------------------|:------------------------|
| **behavior**      | simple AD example       |
| **drive_control** | robot ros driver        |
| **launch**        | default launch files    |
| **msgs**          | robot specific messages |
| **rviz**          | rviz control panel      |
| **service**       | systemd service package |
| **teleop**        | joystick manipulation   |

By default, most of the nodes run with 30 Hz frequency.

## Prerequisites
#### ROS

Please, install **ROS melodic** on **Ubuntu** following the instruction:

http://wiki.ros.org/Installation/Ubuntu

#### RC110 Source

```
export ROS_DISTRO=melodic
source /opt/ros/${ROS_DISTRO}/setup.bash
mkdir -p ~/ros/src && cd ~/ros

cd ~/ros/src
git clone <url or this repo>  # or unzip from archive

cd ~/ros/src/robocar110_ros/
```

#### Build Tools

```
make tools
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
* It will build rviz plugin only in the current ros workspace.

```
source ~/ros/devel/setup.bash
```
* to use it in RViz.

## ROS configuration

Configuration is installed during packages installation.

If you want to install robot config without installing packages, run:
```
make config
```
* It will install default config files to `/etc/zmp/rc110_config/`
