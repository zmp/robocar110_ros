# RC 1/10 Drive Control

## Subscribed Topics

```
/drive [ackermann_msgs::AckermannDrive]
    set speed and steering angle
```

## Published Topic

```
/drive_status [geometry_msgs::TwistStamped]
    actual speed and angle
```

## Parameters

```
rs232_device (string, default: /dev/ttyUSB1)
    motor device name

rs485_device (string, default: /dev/ttyUSB0)
    steering device name
```