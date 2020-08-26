# RoboCar 1/10 ROS nodes

## Dependencies

* ROS
* robocar110

```
# create ROS workspace
mkdir -p ~/ros/src && cd ~/ros/
catkin build
cd src

# clone repo
```

## Build

```
source /opt/ros/<dist>/setup.bash
catkin_make
```

## Package

```
cd ~/ros/build/<package_dir>
make package
```

## Robot configuration
#### ROS

Configuration is installed during packages installation.

If you want to install robot config without installing packages, run:
```
make config
```
* It will install default config files to `/etc/zmp/rc110_config/`

#### Network

To setup wifi connection, check your router SSID and Key and run:
```
sudo rc110-wifi
```
