# Windows Instructions
The ROS installation on windows is not fully supported by ROS itself, only major nodes are available. Please, see the [**official instruction**](http://docs.ros.org/en/rolling/Installation/Windows-Install-Binary.html).

## Makefile
Please, use ROS terminal created in the installation instruction above. I.e. `x64 Native Tools Command Prompt for VS`.
```
set CL=/MP   # enable multicore build
nmake rviz   # build rviz plugin
nmake show   # show rviz
nmake clean  # clean build
```

## Dependencies
Since not all dependencies are available in chocolatey, you need to clone them manually to `%HOMEPATH%\ros\src` (i.e. the same workspace you've cloned this repo).

For example,
```
cd %HOMEPATH%\ros\src\robocar110_ros
nmake rviz
#...
  Could not find a package configuration file provided by "ackermann_msgs"
```

From this message you can understand that `ackermann_msgs` package is missing.
```
cd %HOMEPATH%\ros\src
git clone https://github.com/ros-drivers/ackermann_msgs.git
```

After adding package sources, try to rebuild again. 

Please, make sure that the branch you are cloning is the same as installed ROS distribution. For some packages you might still need to fix the code, so it could be built.

## Alternative Installations
Instead of Chocolatey + MSVC, you can use GCC inside WSL2 or VirtualBox. The downside is they require more computation power and maybe more advanced network installation.
