# Details about Nodes Build

RC110 `make` commands that build the samples use `catkin` under the hood. Here is how it works.

## Dependencies
Before building source code, you need to install all the dependencies. The standard way to do it in ROS is [**rosdep**](http://wiki.ros.org/rosdep).
```
make init-deps  # to prepare rosdep
rosdep install -iry --from-paths .
```

* Some dependencies still might not be found, so you will need to install them manually with `apt install`, etc.

## Build
It's possible to build everything except blacklisted nodes.
```
catkin config --blacklist rc110_video_server rc110_object_detection
catkin build -DCATKIN_ENABLE_TESTING=OFF
```

Or if your IDE supports only single `CMakeLists.txt`, it's better to use `catkin_make`, which will create a root `CMakeLists.txt` containing all the ROS workspace nodes.
```
catkin_make -DCATKIN_BLACKLIST_PACKAGES="rc110_video_server;rc110_object_detection" -DCATKIN_ENABLE_TESTING=OFF
```

## Reference
Please, check the usage of these commands with:
```
catkin build -h
catkin_make -h
```
