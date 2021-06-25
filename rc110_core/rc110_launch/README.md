# RC 1/10 Launch Files #
***

## Sensors ##
***

### IMU ###
***

- Current IMU in robocar110 consists of a single-axis gyroscope and an accelerometer. We use [imu_complementary_filter](https://github.com/ccny-ros-pkg/imu_tools/tree/melodic/imu_complementary_filter) to integrate the two sensors to estimate orientation.

#### Subscribed Topics
```text
/imu/data_raw [sensor_msgs::Imu]
    raw imu data published with frame_id imu_link
```

#### Published Topics
```text
/imu/data [sensor_msgs::Imu]
    integrated imu data with orientation
```

### Camera ###
***

- Please follow this [Monocular Camera Calibration](docs/monocular_camera_calibration.md) to obtain calibration parameters. This calibration step is optional.
- The default path of calibration params for front camera is set in *camera_info_url* of [this configuration file](config/sensors/front_camera.yaml); and optionally, [here](config/sensors/rear_camera.yaml) for rear camera.

- ROS system will give a warning if the calibration files do not exist, which is an expected behavior and can be ignored.
