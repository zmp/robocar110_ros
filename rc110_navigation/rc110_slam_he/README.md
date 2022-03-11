# SLAM with SLAM with Hector Mapping

Hector Mapping can do SLAM basing only on 2D lidar.

## UI
![](docs/slam_he.gif)

## Hector Mapping
#### Project repo
* https://github.com/tu-darmstadt-ros-pkg/hector_slam

#### Documentation
* http://wiki.ros.org/hector_mapping

## Makefile
```
make deps         # check dependencies
make              # build
make run          # run slam node
make show         # show rviz
```

Default map is a rectangle with 50m side and the robot in the middle. If it goes outside the map, hector mapping can break, so it's better to setup a size that you need.
```
make run map_size=4000 map_resolution=0.025   # 4000 * 0.025 = 100m
make run map_size=4000                        # same
```

To save the map, while running SLAM node, execute:
```
make save-map-he map_name=map
```

* Parameters are optional. If skipped, default values used.

## Mapping
* Build and run slam as written above.
* Then drive robot around slowly with joystick.
* You should see a map and localization points.
* Save the map when it's ready.
