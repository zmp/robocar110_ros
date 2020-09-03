# RoboCar 1/10 ROS nodes

## Dependencies

* ROS melodic
* robocar110

## Prepare source
Create ROS workspace
```
source /opt/ros/melodic/setup.bash
mkdir -p ~/ros/src && cd ~/ros
catkin build
```
Clone repo
```
cd src/
git clone <url> 
```

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
* It will install those files with apt.

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
