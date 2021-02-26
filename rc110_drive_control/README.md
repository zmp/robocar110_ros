# RC 1/10 Drive Control

ROS driver node. Please, communicate with it, if you want to get sensor information or drive the robot.

## Subscribed Topics

```
/drive [ackermann_msgs::AckermannDrive]
    set speed and steering angle
```

## Published Topic

```
/drive_status [ackermann_msgs::AckermannDriveStamped]
    actual speed and angle published with frame_id base_link

/imu/data_raw [sensor_msgs::Imu]
    raw imu data published with frame_id imu_link

/servo_temperature [sensor_msgs::Temperature]
    steering servo temperature

/baseboard_temperature [sensor_msgs::Temperature]
    baseboard temperature

/motor_battery [sensor_msgs::BatteryState]
    motor battery state
```

## Parameters

```
rs232_device (string, default: /dev/ttyUSB1)
    motor device name

rs485_device (string, default: /dev/ttyUSB0)
    steering device name

steering_offset (double, default: 0.0)
    steering angle calibration error in degrees
```
