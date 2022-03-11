# RoboCar 1/10 ROS

Here you can find a RoboCar 1/10 ROS driver and sample nodes sources demonstrating its basic usage. 
Releases also contain binary C++ Base Drivers which are used by the ROS nodes. 

[![](docs/images/robocar110x_360p.png)](https://www.zmp.co.jp/en/products/robocar/robocar-110x)

## Contents

| Group                                              | Description                   |
|:---------------------------------------------------|:------------------------------|
| [**rc110_core**](rc110_core/README.md)             | Common Nodes                  |
| [**rc110_monitoring**](rc110_monitoring/README.md) | Remote Monitoring and Control |
| [**rc110_navigation**](rc110_navigation/README.md) | SLAM and Navigation           |
| [**rc110_perception**](rc110_perception/README.md) | Perception                    |
| [**rc110_robot**](rc110_robot/README.md)           | Real Robot Nodes              |
| [**rc110_simulation**](rc110_simulation/README.md) | Simulation                    |

## Binary Packages [Robot Only]
Connect peripherals to the robot according to the User Manual. Boot, login, open browser, go to [**Releases**](https://github.com/zmp/robocar110_ros/releases) and download the latest `*.run` files.

### Base Drivers Installation
```
cd ~/Downloads
sh rc110_drivers_*.run
```

### ROS and RC110 Core + Robot Nodes Installation
```
sh rc110_robot_*.run
```

### System Check
Click on menu button and run "RoboCar 1/10" application.

![](docs/images/rviz.gif)

## Source Code
If you already have `~/ros/src/robocar110_ros/` directory, read [**Sources Update**](docs/SourcesUpdate.md) instead.
```
mkdir -p ~/ros/src
cd ~/ros/src
git clone https://github.com/zmp/robocar110_ros.git
```
```
cd ~/ros/src/robocar110_ros/  # make commands below are called from here!
```

* It's possible to use other directory (for example `~/projects/robocar110_ros/`), but it's not ROS standard way, so we don't provide support for it. Do it on your own risk.

### Check Versions
```
make version
```

* **Tegra:** full operation system version string
* **JetPack (deb):** jetpack version installed from debian package
* **Base Driver:**   version of `rc110_drivers_*.run`
* **ROS Nodes:**     version of nodes listed in `rc110_robot_*.run`
* **Source Code:**   version of this source code

### Robot Nodes
For this, you need a real RoboCar 1/10X. If you don't have one, try simulation described below.

* [**ROS Installation**](docs/RosInstallation.md)
* [**Robot Sources**](rc110_robot/README.md) 

### Remote Connection
Remote connection to the robot is possible from Ubuntu 18 system.

* [**ROS Installation**](docs/RosInstallation.md)
* [**Remote Connection**](docs/RemoteConnection.md)

### Gazebo Simulation
Without robot, there's no need to build driver and system service packages.

* [**ROS Installation**](docs/RosInstallation.md)
* [**Simulation Sources**](rc110_simulation/rc110_gazebo/README.md)

### Other Samples
When the driver is up and running, it's possible to start other ROS nodes that communicate with it.

* [**rc110_behavior**](rc110_navigation/rc110_behavior/README.md)
* [**rc110_slam_he**](rc110_navigation/rc110_slam_he/README.md)
* etc...


## Contribution

* For a completely different node, please, create and use a separate git repo on the same folder level as **robocar110_ros**. Please, tell us, if you developed an interesting application working with the robot!
* For improvements and bugfixes, it's better to modify **robocar110_ros**. See: [**Contribution Process**](docs/Contribution.md). We are open to your feedback!

## Additional Information

* [**Windows Support**](docs/Windows.md)
* [**Make Commands Explanation**](docs/Makefiles.md)
* [**Details about Nodes Build**](docs/BuildDetails.md)
* [**Details about Runtime**](docs/RuntimeDetails.md)
* [**Multiple Robot Setup**](docs/MultiRobot.md)
