# RoboCar 1/10 Service Files

Systemd service files allow to start robot during system startup.

* By default, `robot.launch` is executed.

## Configuration

You can find configuration in `~/.config/rc110/service.conf`

* Please, specify your ip as `ROS_IP` in the file to communicate with the robot through network.
* Also, you can change the default launch command through `RC110_LAUNCH_COMMAND` variable.

## Manipulation Commands
```
systemctl --user status rc110
systemctl --user stop rc110
systemctl --user start rc110
systemctl --user restart rc110
```
