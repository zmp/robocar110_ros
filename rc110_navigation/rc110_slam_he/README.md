# SLAM with SLAM with Hector Mapping

Hector Mapping can do SLAM basing only on 2D lidar.

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
make monitor      # show rviz on remote pc
```

## Mapping
* Build and run slam as written above.
* Then drive robot around slowly with joystick.
* You should see a map and localization points.
