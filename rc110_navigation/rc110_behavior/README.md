# RC 1/10 Behavior Example Node

The node is a simple example of manipulating the robot basing on information from the front lidar using [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP).

The logic is as follows:
* If there are points in a small rectangle before the robot - stop.
* If there are points in a bigger rectangle after that - turn left or right.
* Else - go straight.

## Makefile
```
make deps         # check dependencies
make              # build
make run          # run the node
make show         # show rviz
make monitor      # show rviz on remote pc
```
* After the node has been run, it's possible to enable/disable it using AD button in RViz plugin or AD button on gamepad.

## Subscribed Topics

```
/scan [sensor_msgs::LaserScan]
    2D lidar point cloud
```

## Published Topic

```
/drive_ad [ackermann_msgs::AckermannDrive]
    speed and steering angle
```

## Parameters

```
tree_file (string, default: tree.xml)
    path to behavior tree configuration file
```
