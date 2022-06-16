# Multiple Robot Setup

There is no problem with LAN multirobot setup in ROS since ROS2 anymore!

## Zeroconf Setup

The zeroconf implementation for Linux is called [**Avahi**](https://en.wikipedia.org/wiki/Avahi_(software)). 

It is not necessary for ROS2, but since it is convenient to use robot names instead of ip, you can try to setup it. 

The default configuration assumes that robot hostname starts with `rc-` prefix. Please, change it as follows (dashes are allowed, underscores are not):
```shell
sudo nano /etc/hostname # rc-<robot-name>
sudo nano /etc/hosts    # 127.0.1.1  rc-<robot-name>
sudo reboot
```
Usually `<robot-name>` is filled with SERIAL NUMBER.

Additionally, Avahi itself can be configured:
```shell
sudo nano /etc/avahi/avahi-daemon.conf
sudo systemctl restart avahi-daemon
```

Current hostname can be checked as:
```shell
avahi-resolve -av 127.0.0.1
```

### Known Issues
#### .local domain gets suffix "-2" sometimes
https://github.com/lathiat/avahi/issues/117
* It can happen when network changes from one router to another. To resolve it manually, please, restart the avahi-daemon as written above.
* Also it happens when there are multiple robots with the same name. Thus it's necessary to change hostname as described above.

#### Hostname cannot be resolved, while avahi works
I.e.
```shell
$ ping somename.local
ping: somename.local: Name or service not known

# but
$ avahi-resolve -n4 somename.local
somename.local  192.168.110.7
```

* The solution: https://github.com/lathiat/nss-mdns
```shell
sudo sh -c 'echo .local > /etc/mdns.allow'
sudo nano /etc/nsswitch.conf
```
* And change hosts line to:
```
hosts:          files mdns4 [NOTFOUND=return] dns
```

## Namespaces Setup
Multiple robot setup requires names of nodes, topics, services and parameters to be prefixed with unique identifier to avoid names collision. Usually robot/machine name is used for this purpose (dashes changed to underscores).

For example with namespace `/robot_name`, instead of `/front_lidar/scan`, the topic will be called `/robot_name/front_lidar/scan`.

[**ROS Transforms**](http://wiki.ros.org/tf2) also need to be prefixed.

In contrast to ros namespace, frame id **must not** start with slash (`/`). Also note that separator for prefix is designed to be **only slash**, because it is used with `tf_prefix` parameter.

For example, instead of `base_link`, the frame id will be called `robot_name/base_link`.
