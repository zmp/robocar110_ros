# RC 1/10 Behavior Example Node

The node is a simple example of manipulating the robot basing on information from the front lidar.

The logic is as follows:
* If there are points in a small rectangle before the robot - stop.
* If there are points in a bigger rectangle after that - turn left or right.
* Else - go straight.

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
tree_file (string, default: behavior.xml)
    path to behavior tree configuration file
```