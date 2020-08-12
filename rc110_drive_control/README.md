# RC 1/10 Drive Control

## Subscribed Topics

```
/drive [ackermann_msgs::AckermannDrive]
    set speed and steering angle
```

## Published Topic

```
/drive_status [ackermann_msgs::AckermannDriveStamped]
    actual speed and angle

/imu [sensor_msgs::Imu]
    imu data

/servo_temperature [sensor_msgs::Temperature]
    steering servo temperature

/motor_temperature [sensor_msgs::Temperature]
    wheel motor temperature

/motor_battery [sensor_msgs::BatteryState]
    motor battery state
```

## Parameters

```
rs232_device (string, default: /dev/ttyUSB1)
    motor device name

rs485_device (string, default: /dev/ttyUSB0)
    steering device name
```