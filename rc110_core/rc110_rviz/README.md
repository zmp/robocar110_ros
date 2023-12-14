# RoboCar 1/10 RViz plugin

## Makefile
```
make deps         # check dependencies
make              # build
make run          # run only plugin without rviz
make show         # show rviz
```

## Note on the serial number

RoboCar 1/10 units come with a serial number.
It is prepended to every node and topic in the ROS system to identify the unit.
However, [RViz config file](rviz/main.rviz) has preset topic names whose serial number may not match the actual unit.
It won't show any lidar data or camera images if you do not fix it.
In that case, follow the instructions below.

First, if you haven't already, source the setup.bash

    $ source install/setup.bash

Second, run the script. It will ask for the number

    $ rc110-set-ros-serial xxxxx

where _xxxxx_ is the serial number.
