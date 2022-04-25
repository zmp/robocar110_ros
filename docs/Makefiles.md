# Make Commands Explanation
## Targets
In this project, we use `make` commands to automate shell scripts: `make deps`, `make`, `make run`, etc. It's done to reduce amount of typing and utilize makefile rule prerequisites feature.

Information about some targets:
```shell
make help
```

The list of all targets can be shown with:
```shell
make <Tab>
```

To know the content of each make target, either execute the command and check its output or open `GNUmakefile` in root / node folder with text editor. (`Makefile` for Windows)

## Alternative
Of course, you can use ros commands directly. For example, `make run` command is described in `run:` paragraph of `GNUmakefile` similar to:
```shell
source $(catkin locate rc110)/host_setup.bash on_robot
roslaunch --wait rc110_system robot.launch  # and args...
```

Then you can check the meaning of those commands in public documentation.

* `source`: https://ss64.com/bash/source.html
* `roslaunch`: http://wiki.ros.org/roslaunch
* `systemctl`: https://www.freedesktop.org/software/systemd/man/systemctl.html

## Specifics of RC110 Makefiles
### Arguments
It's possible to pass variables to `make` as following:
```shell
make my_target var1=a var2=b
```

Internally some targets use `ros_args` macros which provides roslaunch arguments out of those variables, so `roslaunch <...> $(ros_args)` becomes:
```shell
roslaunch <...> var1:=a var2:=b
```

Roslaunch discards unused arguments, so it's probably ok to mix makefile and roslaunch names.

## Required Knowledge

* `bash`: https://en.wikibooks.org/wiki/Bash_Shell_Scripting
* `make`: https://www.gnu.org/software/make/manual/html_node/index.html