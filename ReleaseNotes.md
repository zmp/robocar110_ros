## v2.2.0
* Add a script to replace serial numbers in RViz configuration file in setup process
* Add missing dependencies to build ROS2 packages into installer script (rc110_robot_*.run)
* Add steering servo motor torque value and setting field to RViz
* Update the default torque value to 80% (previously 3%)

## v2.1.0
* NVIDIA video streamer has been replaced with usb_node for visualization in RVIZ to avoid issue where stream doesn't appear.

## v2.0.0
* RoboCar 1/10 software has been retrofitted to run on ROS 2 (Rolling)

### Actions
* Install `ROS 2` with distribution `Rolling`.
### Notes
* Simulation and large parts of navigation (rc110_slam_he, rc110_slam_cg) do not exist due to incompatibility with ROS 2.
* rc110_gazebo and rc110_navigation nodes will be added at a later date due to need for futher development to be compatible with ROS 2.

## v1.12.0 ~ .1
* Switch selected robot with topics instead of parameter.
* Fix ros installation script.
* RViz video panel for playing streams (only remote).
* Multiple joysticks support.
* Fix ROS source bug which arises from emtpy ROS_DISTRO variable

## v1.11.0
* Goal queue allows setting consequent goals in [navigation](rc110_navigation/rc110_navigation/README.md).
* Switched to `urg_node` for smoother update in the future.
* Hokuyo driver was added to `rc110_drivers_*.run`.
* Checked separate cartographer installation on Ubuntu 20.

### Actions
* Install `rc110_drivers` with version `>=1.4.0`.

## v1.10.0
* Video server node is included into the default robot installation. It is possible to use it without stopping camera node, but not simultaneously with it.
* GUI robot selector now allows to omit option `rc=rc_12345`.
* Cartographer package is not released on noetic repo for the time being. So it's necessary to compile the cartographer manually for `rc110_slam_cg`, or just use `rc110_slam_he` instead.

### Actions
* **[noetic]** Camera driver config was updated. Please, install the latest `rc110_drivers.run` and launch `rc110-camera` again.
* **[noetic]** Before compiling, please, install dependencies with `make deps force=on`.

### Notes
* Build on Ubuntu 20 with ROS noetic was checked, but only Ubuntu 18 with ROS melodic is officially supported for the moment.
* JetPack >=5 with Ubuntu 20 is not supported with this version.

## v1.9.0 ~ .1
* Multiple robots setup with ros master on each machine (FKIE).
* Automatic network discovery (on top of [**avahi**](https://en.wikipedia.org/wiki/Avahi_(software))).
* Joystick switching between robots.
* Automatic joystick type detection.
* Merged `rc110_launch` into `rc110_system`, renamed `rc110_common` to `rc110`.
* Removed `rc110_master_hold`, as multimaster does reconnection instead.
* `make monitor` commands are fully replaced with `make show`.
* Makefile parameters use `=` only, not `:=`.
* Renamed `~/.config/rc110/service.conf` to `config.bash`.
* Navigation follows global path more precisely.

### Actions
* Change robot hostname as described [**here**](docs/MultiRobot.md).

## v1.8.0
* Basic model training documentation for pytorch ssd.
* Simulation worlds: "Maze", imported worlds.
* Mouse teleoperation.
* Build improvements.

## v1.7.0 ~ .1
* MATLAB support (requires [MATLAB Connection Option](https://www.zmp.co.jp/en/products/robocar/robocar-110X/support/matlab)).
* API change: `/motor_state` and `/servo_state` from  `std_srvs::SetBool` to `rc110_msgs::SetInteger`.
* Wheel joystick support (`make remote-teleop joy_type:=logicool`).
* Offline rosdep configuration (`make init-deps-offline`).
* Simplified behavior node.
* Improved cartographer slam configuration, IMU usage.
* Cartographer map saving (`make save-map-cg map_name=map_cg`).
* Simulation fixes.

## v1.6.0
* Documentation improvement (make commands explanation, gifs, etc)
* Better installer with run file.
* Remote monitoring ROS_IP fix.
* ROS installation fix (`make ros-install`).

## v1.5.0
* Object detection with Darknet Yolo.
* Simulation with Gazebo.

## v1.4.0
* Navigation on the map created with Hector SLAM.
* rc110_behavior is removed from core nodes.
* Driver v1.1.0. Few steering improvements for better odometry.

## v1.3.0
* 2D object detection with jetson_inference.
* Convenient camera calibration.
* h265 video streaming (around 10 times faster than ROS image).
* Remote joystick control.
* Improved hierarchy of package directories.

## v1.2.0
* SLAM samples based on Cartographer and Hector.
* Tested with JetPack 4.5.1

## v1.1.0
* Setting constant ROS_IP in configuration.
* Separate RViz panel build.
* New message type for baseboard errors.

## v1.0.0
* Initial version.
* ROS control nodes.
* RViz panel.
* Service files.
