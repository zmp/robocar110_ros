# Details about Nodes Build

RC110 `make` commands that build the samples use `colcon` under the hood. Here is how it works.

## Dependencies
Before building source code, you need to install all the dependencies. The standard way to do it in ROS is [**rosdep**](http://wiki.ros.org/rosdep).
```
make init-deps  # to prepare rosdep
rosdep install -iry --from-paths .
```

* Some dependencies still might not be found, so you will need to install them manually with `apt install`, etc.

## Build
```
colcon build --symlink-install --packages-up-to rc110_system
```

* `--symlink-install` - avoid building when only scripts are changed
* `--packages-up-to` - build listed packages and their dependencies

## Reference
Please, check the usage with:
```
colcon build -h
colcon -h
```
