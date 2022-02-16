# RC 1/10 Drive Control

ROS driver node. This node receives drive control commands and publishes sensor information from the robot.

## Subscribed Topics

```
/drive [ackermann_msgs::AckermannDriveStamped]
    set speed and steering angle
    
/offsets [rc110_msgs::Offsets]
    various device value offsets
```

## Published Topic

```
/motor_speed_goal [std_msgs::Float32]
    drive motor target speed (m/s)

/steering_angle_goal [std_msgs::Float32]
    steering target angle (rad)
    
/baseboard_error [rc110_msgs::BaseboardError]
    Error from baseboard

/robot_status [rc110_msgs::Status]
    Status of baseboard, drive and steering motors

/drive_status [ackermann_msgs::AckermannDriveStamped]
    steering angle and drive speed published on "base_link" frame id
    , the speed is the same as /motor_rate estimated_speed

/offsets_status [rc110_msgs::Offsets]
    current devices offsets values

/imu/data_raw [sensor_msgs::Imu]
    raw imu data published with frame_id imu_link

/servo_temperature [sensor_msgs::Temperature]
    steering servo temperature

/baseboard_temperature [sensor_msgs::Temperature]
    baseboard temperature

/servo_battery [sensor_msgs::BatteryState]
    baseboard battery state measured by steering motor
    
/motor_battery [sensor_msgs::BatteryState]
    baseboard battery state measured by drive motor
    
/odometry [nav_msgs::Odometry]
    odometry calculated basing on motor rate and steering info
    
/motor_rate [rc110_msgs::MotorRate]
    auxiliary drive motor rotation rate and robot speed estimated from it
    , the speed is the rate * gears factor * PI * wheel diameter
    
/wheel_speeds [rc110_msgs::WheelSpeeds]
    separate speeds of each wheel measured by wheel encoders
```

## Services

```
enable_board [std_srvs::SetBool]
    enable/disable baseboard
    
motor_state [rc110_msgs::SetInteger]
    drive motor: off / on / neutral
    
servo_state [rc110_msgs::SetInteger]
    steering motor: off / on / neutral
```

## Parameters

```
base_frame_id (string, default: base_link)
    base frame id

imu_frame_id (string, default: imu_link)
    imu frame id

rate (double, default: 30)
    node refresh rate (Hz)

odometry_only_angle_offset (double, default: 1.0)
    steering angle absolute value is reduced by this number when odometry is calculated (Deg)
    change the value basing on the experimental data of your environment
    http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide#Odometry
```
