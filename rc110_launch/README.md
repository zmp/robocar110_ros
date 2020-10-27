# RC 1/10 Launch Files #
***

## Sensors ##
***

### IMU ###
***

- Current IMU in robocar110 consists of a single-axis gyroscope and an accelerometer. We use [imu_complementary_filter](https://github.com/ccny-ros-pkg/imu_tools/tree/melodic/imu_complementary_filter) to integrate the two sensors to estimate orientation.

- Install imu_tools:
```
    sudo apt-get install ros-${ROS_DISTRO}-imu-tools
```

- Subscribed Topics:
```text
/imu/data_raw [sensor_msgs::Imu]
    raw imu data published with frame_id rc110_imu
```

- Published Topics:
```text
/imu/data [sensor_msgs::Imu]
    integrated imu data with orientation
```
