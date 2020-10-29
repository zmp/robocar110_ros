# RoboCar 1/10 ROS nodes for teleoperation

## Dependencies

- ROS environment
- joystick ROS drivers
```bash
    sudo apt-get install ros-${ROS_DISTRO}-joy
    sudo apt-get install ros-${ROS_DISTRO}-joystick-drivers
```

## Controls

- [Details of the Joystick](https://www.elecom.co.jp/products/JC-U4113SBK.html)
![Joystick Structure](./docs/images/joystick.jpg)


- Button 5: press and hold it to drive
- Button 7: increases gear (gear 1 on start)
- Button 8: decreases gear
- Axis 9:  up and down regulates speed
- Axis 10: left and right changes steering angle

#### Gears

1. Up to 3 km/h
2. Up to 6 km/h
3. Maximum speed

## Remapping

To know buttons and axes indexes, use `rostopic echo /joy`

Buttons mapping is set in `joy_teleop.yaml`.
* To set axis inversion, set negative index.

**NB:** AUTO and CLEAR are not mapped by ROS driver.
