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
