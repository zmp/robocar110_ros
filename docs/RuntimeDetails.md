# Details about Runtime
## ROS2 Sourcing
Basically there two types of usual ROS2 environment:
* System: `source /opt/ros/${ROS_DISTRO}/setup.bash`
* Install: `source install/setup.bash`

The first one adds environment variables for packages installed to `/opt` folder.
The other one is the same as first, but also with packages deployed to `install` folder of ROS workspace.

Please, note that by default ROS2 gives warning, if you try to build packages after sourcing `install` folder, or any other case when built packages are already installed. To silence it, we use `--allow-overriding` trick.

The ROS environment can be checked with the following command:
```shell
env | grep ROS
```
