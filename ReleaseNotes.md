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
