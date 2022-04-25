# Details about Runtime
## ROS Sourcing
Basically there two types of usual ROS environment:
* System: `source /opt/ros/${ROS_DISTRO}/setup.bash`
* Devel: `source $(catkin locate --devel)/setup.bash`

The first one adds environment variables for packages installed to `/opt` folder.
The other one is the same as first, but also with packages built in `devel` folder of ROS workspace.

We add another level of sourcing by starting **roscore** with synchronization in background task, adding network environment and providing ROS namespace:
```shell
source $(catkin locate rc110)/host_setup.bash  # hostname namespace
# or
source $(catkin locate rc110)/auto_setup.bash  # automatic robot namespace
```

The ROS environment can be checked with the following command:
```shell
env | grep ROS
```

To finish the roscore, just close the terminal.

## Robot Selection
If you are running multiple robots (real or simulation), by default the first robot is selected. It can be changed with `rc` variable:
```
cd ~/ros/src/robocar110_ros

make show rc=grc_2
make run-navigation rc=grc_2
# etc...
```
