# RC 1/10 Simulation Sample Package

This package contains components for simulation of RC 1/10 in Gazebo with drive control functionality and sensors output using open-source Gazebo ROS plugins.

# Nodes
## RC 1/10 Simulation Interface

Node for converting RC 1/10 AckermannDrive message to Twist message that is supported by the Gazebo Model plugin. AckermannDrive speed and steering angle is converted to Twist linear X and angular Z, respectively.

### Subscribed Topics

```
/drive [ackermann_msgs::AckermannDriveStamped ]
    input ackermann drive message
```

### Published Topic

```
/drive_twist [geometry_msgs::Twist]
    ackermann drive converted to twist message
```
