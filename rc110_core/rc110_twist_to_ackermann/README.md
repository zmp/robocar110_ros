# RC 1/10 Twist to Ackermann

Since ROS navigation stack does not support Ackermann message, the conversion should be done.

## Subscribed Topics

```
cmd_vel [geometry_msgs::Twist]
    velocity message from navigation node
```

## Published Topic

```
drive_ad [ackermann_msgs::AckermannDriveStamped ]
    ackermann message
```
