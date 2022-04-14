# RoboCar 1/10 Service Files

Systemd service files allow to start robot during system startup.

* By default, `robot.launch` is executed.

## Configuration

You can find configuration in `~/.config/rc110/config.bash`

* It's possible to chage roslaunch arguments with `RC110_ARGS` variable.
* Also, you can change the default launch command through `RC110_LAUNCH_COMMAND` variable.

## Manipulation Commands
```
systemctl --user status rc110
systemctl --user stop rc110
systemctl --user start rc110
systemctl --user restart rc110
```
Similar effect can be acheived with `make start`, `make stop`, etc shortcuts.

## Sensors
### IMU
* Current IMU in robocar110 consists of a single-axis gyroscope and an accelerometer. We use [imu_complementary_filter](https://github.com/ccny-ros-pkg/imu_tools/tree/melodic/imu_complementary_filter) to integrate the two sensors to estimate orientation.

#### Subscribed Topics
```
/imu/data_raw [sensor_msgs::Imu]
    raw imu data published with frame_id imu_link
```

#### Published Topics
```
/imu/data [sensor_msgs::Imu]
    integrated imu data with orientation
```

### Camera
* Please follow [**Camera Calibration**](docs/CameraCalibration.md) instruction to obtain calibration parameters. This calibration step is optional.
* The default path of calibration params for front camera is set in *camera_info_url* inside the [**configuration file**](config/sensors/front_camera.yaml) (and optionally, [**here**](config/sensors/rear_camera.yaml) for rear camera).
* ROS system will give a warning if the calibration files do not exist, which is an expected behavior and can be ignored.
