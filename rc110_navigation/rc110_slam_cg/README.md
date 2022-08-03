# SLAM with Google Cartographer

Current configuration allows creating a map basing on 2D lidar.

## UI
![](docs/slam_cg.png)

## Google Cartographer
#### Project repo
* https://github.com/cartographer-project/cartographer

#### Documentation
* https://google-cartographer-ros.readthedocs.io/en/latest/

## Makefile
```
make deps         # check dependencies (for Ubuntu 20 see below)
make              # build
make run          # run slam node
make show         # show rviz
```

To save the map, while running SLAM node, execute:
```
make save-map-cg map_name=map map_resolution=0.025
```

* Parameters are optional. If skipped, default values used.

## Mapping
* Build and run slam as written above.
* Then drive robot around slowly with joystick.
* You should see a map and localization points.
* Save the map when it's ready.

## Installing Cartographer from Source
Here is the rough instruction on how to install the cartographer, if you don't have packages in APT. Please, check the official documentation for additional information.

```
cd ~/ros/src/
git clone https://github.com/abseil/abseil-cpp.git
git clone https://github.com/cartographer-project/cartographer.git
git clone https://github.com/cartographer-project/cartographer_ros.git

cd abseil-cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON
make -j
sudo checkinstall -y --pkgname absl-zpkg

cd ~/ros/src/
rosdep install -iry --from-paths cartographer cartographer_ros
catkin build cartographer_ros
```
