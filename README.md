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

We recommend to build the [**core packages**](rc110_core/README.md#Build) first, and then each additional node one by one.

If you want to build all the packages at once, try the following:
```
make init  # to prepare rosdep
rosdep install -iry --from-paths .
```
* Then install manually dependencies that rosdep does not support.
* And run the build:
```
catkin build -DCATKIN_BLACKLIST_PACKAGES=rc110_video_server;rc110_object_detection
```
* `CATKIN_BLACKLIST_PACKAGES` variable above allows to skip nodes you don't need.

## ~ Additional ~

* [**Windows**](docs/Windows.md)
* [**Contribution Rules**](docs/Contribution.md)
