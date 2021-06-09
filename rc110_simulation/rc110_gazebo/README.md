# RC 1/10 Gazebo Simulation Sample

This package contains components for simulation of RC 1/10 in Gazebo with drive control functionality and sensors output using open-source Gazebo ROS plugins.

* Conversion of RC 1/10 AckermannDrive message to Twist message that is supported by the Gazebo Model plugin.
* Publishing information similar to **rc110_drive_control**.

## Makefile

```
make deps         # check dependencies
make              # build
make run          # run simulation
make show         # show rviz  (At first from robocar110_ros/, you need to run: make rviz)

make run use_gui:=false    # run without gazebo gui
```

## Subscribed Topics

```
/drive [ackermann_msgs::AckermannDriveStamped ]
    input ackermann drive message
```

## Published Topics

```
/drive_twist [geometry_msgs::Twist]
    ackermann drive converted to twist message
```

## World Creation

* See the official tutorial: http://gazebosim.org/tutorials?cat=build_world&tut=building_editor
* Roslaunch parameter for custom world: `make run world:=/path/to/custom.world`

## Issues

* `gazebo died from signal 9` or `[gazebo-5] escalating to SIGTERM`
    * Gazebo bug: https://github.com/ros-simulation/gazebo_ros_pkgs/issues/751

* `[Err] [OpenAL.cc:84] Unable to open audio device[default]`
    * Few seconds delay on virtual pc due to lack of sound card.
    * Sometimes it does not cause delay.

* `[Err] [ModelDatabase.cc:390] Unable to parse model.config for model`
    * Workaround: `export GAZEBO_MODEL_DATABASE_URI=" "`
    * See: https://github.com/osrf/gazebo/issues/2934
    * You'll get instead: `[Wrn] [ModelDatabase.cc:212] Unable to connect to model database using [ //database.config].`

* `[Wrn]` prefixed messages can be ignored probably.
