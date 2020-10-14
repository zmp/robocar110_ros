# RoboCar 1/10 ROS nodes for teleoperation #
***

## Dependencies ##
***

- ROS environment
- joystick ROS drivers
```bash
    sudo apt-get install ros-${ROS_DISTRO}-joy
    sudo apt-get install ros-${ROS_DISTRO}-joystick-drivers
```

## References ##
***

- [Details of the Joystick](https://www.elecom.co.jp/products/JC-U4113SBK.html)
![Joystick Structure](./docs/images/joystick.jpg)

- To control rc110 by joystick, first press button number 5 on the joystick; direct the axis (at the 10th position) up and down for steering angle; direct the axis (at the 9th position) up and down for speed.

**連射AUTOボタン and 連射CLEARボタン are not mapped by ROS drivers**
