# Details about Runtime
## ROS Sourcing
Basically there two types of usual ROS environment:
* System: `source /opt/ros/${ROS_DISTRO}/setup.bash`
* Devel: `source $(catkin locate --devel)/setup.bash`

The first one adds environment variables for packages installed to `/opt` folder.
The other one is the same as first, but also with packages built in `devel` folder of ROS workspace.

We add more levels of sourcing, each of them include previous level.

* With RC110 environment: `source /opt/ros/${ROS_DISTRO}/rc_setup.bash`
* With multimastered roscore: `source $(catkin locate rc110)/host_setup.bash`
* With automatic robot selection: `source $(catkin locate rc110)/auto_setup.bash`

The ROS environment can be checked with the following command:
```shell
env | grep ROS
```

To finish the roscore, just close the terminal.
