# Details about Runtime
## ROS Sourcing
Basically there can be two types of ROS environment to source:
* System: `source /opt/ros/${ROS_DISTRO}/setup.bash`
* Devel: `source $(catkin locate --devel)/setup.bash`

The first one adds environment variables for packages installed to `/opt` folder.
The other one is the same as first, but also with packages built in `devel` folder of ROS workspace.

We add another level to Devel sourcing by adding network environment:
```shell
source $(catkin locate rc110)/env/devel.bash
```

The ROS environment can be checked with the following command:
```shell
env | grep ROS
```

## Multimaster Launch Files
Usual command to call a launch script is:
```shell
roslaunch package script arg1:=x arg2:=y
```

But for multimaster system we need to run also synchronization nodes. Thus the command above transforms into:
```shell
rosrun rc110 launch package script arg1:=x arg2:=y
```

Similarly for `rosmon`:
```shell
rosrun rc110 mon package script arg1:=x arg2:=y
```

If roscore is already running, the `rosrun rc110` commands won't run it again. They assume that roscore always starts with multimaster synchronization.