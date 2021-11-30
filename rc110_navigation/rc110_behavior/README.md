# RC 1/10 Behavior Example Node

The node is a simple example of manipulating the robot basing on information from 2D lidars.

Please, see [implementation](src/rc110_behavior.hpp) for more details.

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
/lidar_cloud [sensor_msgs::PointCloud2]
    3D point cloud combined from front and rear lidars
```

## Published Topic

```
/drive_ad [ackermann_msgs::AckermannDrive]
    speed and steering angle
```

## Parameters

```
forward_command (vector, default: [0.4, 0])
    forward speed and steering
    
left_command (vector, default: [0.25, 28])
    left turn speed and steering
    
right_command (vector, default: [0.25, -28])
    right turn speed and steering
```
