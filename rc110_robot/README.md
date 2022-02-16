# Nodes Used on Robot Only

They are preinstalled on robot and run with systemd service.

| Name                                                | Description                   |
|:----------------------------------------------------|:------------------------------|
| [**drive_control**](rc110_drive_control/README.md)  | Robot ros driver              |
| [**launch**](rc110_launch/README.md)                | Default launch files          |
| [**system**](rc110_system/README.md)                | Systemd service package       |
| [**hokuyo_node**](http://wiki.ros.org/hokuyo_node)  | Hokuyo 2D Laser Driver        |

## Makefile
```
cd ~/ros/src/robocar110_ros
```

To install a new version of ROS nodes:
```
make deps     # ros dependencies
make package  # creates deb files in the build directory
make install  # calls: sudo apt-get install
make start    # restarts system nodes
```

To run nodes without installation:
```
make deps     # ros dependencies
make          # build robot packages
make run      # run nodes from build folder
```

## Remote Configuration

In order to connect to robot from remote PC, `ROS_MASTER_URI` and `ROS_IP` environment variables should be set. `env.sh` file is created automatically for this purpose to use with `make` command.

If you want to configure those variables beforehand, please, run the following command and adjust the variables.
```
make env
```

## ROS configuration

Default configuration is installed during packages installation.

If on update the configuration was changed in both new package and your local file, you will be prompted to adjust it manually.

* `/opt/ros/${ROS_DISTRO}/share/rc110_launch/config/`
