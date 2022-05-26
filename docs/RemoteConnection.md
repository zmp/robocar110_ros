# Remote Connection

After [**installing ROS**](RosInstallation.md), please try to execute the following commands. 

## Makefile
```
make deps-remote  # check dependencies
make remote       # build
make show         # show rviz on remote PC
make run-teleop   # connect joystick from remote PC
make node-manager # GUI to manage robots
... etc
```

## Robot Selection
If you are running multiple robots (real or simulation), by default the first robot is selected. It can be changed with `rc` variable:
```
make show rc=rc_21000
make run-navigation rc=rc_21000
# etc...
```

## Notes
* A button for switching robot was added to [**joystick configuration**](../rc110_core/rc110_teleop/README.md).
* First "execution" command will also run synchronization nodes. So it's convenient to keep sync with for example `run-teleop` and then run/finish other scripts without restarting synchronization.

## Details
If you are wondering how it works or need to setup more than one robot, check the [**Multiple Robot Setup**](MultiRobot.md) document.
