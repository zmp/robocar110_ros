# Make Commands Explanation

In this project we use `make` commands to automate shell scripts: `make deps`, `make`, `make run`, etc...

The list of all targets can be shown with:
```shell
make <Tab>
```

To know the content of each make target, either execute the command and check its output or open `GNUmakefile` in root / node folder with text editor. (`Makefile` for Windows)

## Alternative
Of course, you can use ros commands directly. For example, instead of `make run`, you can execute:
```shell
systemctl --user start rc110-roscore
source ../../devel/setup.bash
source ~/.config/rc110/service.conf
eval "$$RC110_LAUNCH_COMMAND"
```
(See `run:` paragraph in `GNUmakefile`)

Which is equivalent to even simpler command:
```shell
source ../../devel/setup.bash
ROS_IP=192.168.110.5
roslaunch rc110_launch robot.launch
```

Then you can check the meaning of those commands in public documentation.

* `source`: https://ss64.com/bash/source.html
* `roslaunch`: http://wiki.ros.org/roslaunch
* `systemctl`: https://www.freedesktop.org/software/systemd/man/systemctl.html

## Required Knowledge

* `bash`: https://en.wikibooks.org/wiki/Bash_Shell_Scripting
