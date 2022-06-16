# Remote Connection

After [**installing ROS**](RosInstallation.md), please try to execute the following commands. 

## Makefile
```
make deps-remote  # check dependencies
make remote       # build
make show         # show rviz on remote PC
make run-teleop   # connect joystick from remote PC
... etc
```

## Robot Selection
If you are running multiple robots (real or simulation), [**robot selector**](../rc110_core/rc110_selector/README.md) is shown. It can be changed with `rc` variable:
```
make show rc=rc_21000
```

## Notes
* A button for switching robot was added to [**joystick configuration**](../rc110_core/rc110_teleop/README.md).

## Details
If you are wondering how it works or need to setup more than one robot, check the [**Multiple Robot Setup**](MultiRobot.md) document.
