# RoboCar 1/10 ROS nodes

It's a set of default ROS nodes for Robocar 1/10.

They are preinstalled on robot and run with systemd service.

| Name              | Description             |
|:------------------|:------------------------|
| **behavior**      | simple AD example       |
| **drive_control** | robot ros driver        |
| **launch**        | default launch files    |
| **msgs**          | robot specific messages |
| **rviz_panel**    | rviz control panel      |
| **service**       | systemd service package |
| **teleop**        | joystick manipulation   |

By default, most of the nodes run with 30 Hz frequency.

## Prerequisites
#### ROS

Please, install ROS melodic following the instruction:

http://wiki.ros.org/Installation/Ubuntu

#### Get Source

```
export ROS_DISTRO=melodic
source /opt/ros/${ROS_DISTRO}/setup.bash
mkdir -p ~/ros/src && cd ~/ros
catkin build

cd ~/ros/src
git clone <url>  # or unzip from archive
cd ~/ros/src/robocar110_ros/
```

#### Dependencies

```
sudo apt install python-rosdep
sudo rosdep init
rosdep update --rosdistro=$ROS_DISTRO

cd ~/ros/src/robocar110_ros/
make deps
```

#### ROS Environment

```
make ros-source
```
* It will place ros sourcing to `~/.bashrc`. 

## Build

```
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

## ROS configuration

Configuration is installed during packages installation.

If you want to install robot config without installing packages, run:
```
make config
```
* It will install default config files to `/etc/zmp/rc110_config/`
