# RoboCar 1/10 ROS

[![](docs/images/robocar110x_360p.png)](https://www.zmp.co.jp/en/products/robocar/robocar-110x)

ROS nodes for RoboCar 1/10 are divided to multiple groups:

| Group                                              | Description                   |
|:------------------                                 |:------------------------      |
| [**core**](rc110_core/README.md)                   | Major Nodes                   |
| [**monitoring**](rc110_monitoring/README.md)       | Monitoring and Remote Control |
| [**navigation**](rc110_navigation/README.md)       | Navigation Nodes              |
| [**perception**](rc110_perception/README.md)       | Perception Nodes              |
| [**utils**](rc110_utils/README.md)                 | Utilities                     |

* The difference between core group and other groups is that Core Nodes are installed by default and tend to have more support than other groups.
* Any node can be customized later. Please, create and use a separate project within the same ROS workspace, instead of modifying inside **robocar110_ros**.

## Prerequisites
#### ROS

Please, install **ROS melodic** on **Ubuntu** following the instruction:

http://wiki.ros.org/Installation/Ubuntu

#### RC110 Source Code

```
mkdir -p ~/ros/src
cd ~/ros/src
git clone <url or this repo>  # or unzip from archive

cd ~/ros/src/robocar110_ros/
```

#### ROS Environment

```
make ros-source
```
* Optional.
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

## Remote Configuration

In order to connect to robot from remote PC, `ROS_MASTER_URI` and `ROS_IP` environment variables should be set. `env.sh` file is created automatically for this purpose to use with `make` command.

If you want to configure those variables beforehand, please, run the following command and adjust the variables.
```
make env
```

## RViz Panel

```
make deps-rviz
make rviz
```
* It will build rviz plugin in the current ros workspace.

```
make show
```
* to show RViz with default layout locally, or

```
make monitor
```
* to show RViz with default layout on remote PC

## Joystick Connected to PC

Connect joystick dongle to your remote PC instead of robot.
```
make remote-joy
```

## ROS configuration

Default configuration is installed during packages installation.

If on update the configuration was changed in both new package and your local file, you will be prompted to adjust it manually.

* `/opt/ros/${ROS_DISTRO}/share/rc110_launch/config/`

## ~ Additional ~

* [**Windows**](docs/Windows.md)
* [**Contribution Rules**](docs/Contribution.md)
