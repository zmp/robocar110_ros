# Nodes Used on Robot Only

They are preinstalled on robot and run with systemd service.

| Name                                                | Description                   |
|:----------------------------------------------------|:------------------------------|
| [**drive_control**](rc110_drive_control/README.md)  | Robot ros driver              |
| [**system**](rc110_system/README.md)                | Systemd service package       |
| [**video_server**](rc110_video_server/README.md)    | RTSP Video Server             |

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

## ROS configuration

Default configuration is installed during packages installation.

If on update the configuration was changed in both new package and your local file, you will be prompted to adjust it manually.

* `/opt/ros/${ROS_DISTRO}/share/rc110_system/config/`
