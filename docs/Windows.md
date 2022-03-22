# Windows Instructions
The ROS installation on windows is not fully supported by ROS itself, only major nodes are available. Please, see the [**official instruction**](http://wiki.ros.org/Installation/Windows).

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

Another example. There's error about image transport in RViz, it's not clear, but it's possible to understand that one package is missing.
```
cd %HOMEPATH%\ros\src
git clone https://github.com/ros-perception/image_transport_plugins.git
```

After adding package sources, try to rebuild again. 

Please, make sure that the branch you are cloning is the same as installed ROS distribution. For some packages you might still need to fix the code, so it could be built.

## Launch Notes
Since avahi is not available on Windows, it would be difficult to configure multirobot setup similar to Ubuntu. So for ROS1, it's limited only to one robot connection from Windows.
```
set ROS_NAMESPACE=/zmp
set ROS_IP=192.168.<windows_ip_from_ipconfig>
set ROS_MASTER_URI=http://zmp.local:11311
```

Also you might need to use some kind of domain name resolution like `C:\Windows\System32\drivers\etc\hosts` file:
```
192.168.110.5 zmp.local
```

After the configuration is done, try to run RViz.
```
nmake show
```

## Alternative Installations
Instead of Chocolatey + MSVC, you can use GCC inside WSL2 or VirtualBox. The downside is they require more computation power and maybe more advanced network installation.
