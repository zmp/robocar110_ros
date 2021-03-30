# Note on Monocular Camera Calibration #
***

The calibration needs to be done to get the intrinsic parameters and distortion coefficients of the camera. They can be applied to get undistorted images.

The `~/image_raw` messages contain the distorted images of the front (or optional rear) camera. To perform the calibration, follow [the instruction](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration).

Suppose you are doing calibration for front camera of the robot, please do the following steps:

- Run calibration node:
```bash
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.35 --camera_name front_camera image:=/front_camera/image_raw camera:=/front_camera
# for calibration board of different size, please change the value of --size and --square
```
- Perform the procedure from the node manual.
- After you click **COMMIT**, the configuration should be saved in:
    - `~/.ros/camera_info/front_camera.yaml`
- Restart robot nodes, and you will get new topics:
    * `~/image_rect_color` (color undistorted image)
    * `~/image_rect` (monochrome undistorted image)


## Installation from Saved File ##

If you clicked **SAVE**, instead of **COMMIT** above, it's still possible to apply the calibration to camera:
```bash
cd ~/ros/src/robocar110_ros/
make camera-calibration-file-front
```

#### Explanation ####

- The calibrated data is saved into `/tmp/calibrationdata.tar.gz`
- First, it is extracted and the yaml file is adjusted:
```bash
tar -xzf /tmp/calibrationdata.tar.gz -C /tmp --one-top-level=calib  # creates ost.yaml file
sed -i 's/camera_model: plumb_bob/distortion_model: plumb_bob/g' /tmp/calib/ost.yaml  # replaces camera_model with distortion_model
```

- Next, the yaml file is put to the correct location:
```bash
mkdir -p ~/.ros/camera_info
mv /tmp/calib/ost.yaml ~/.ros/camera_info/front_camera.yaml
# for rear camera, after rear camera calibration, change front_camera.yaml to rear_camera.yaml in the above command line.
```

You can check which images were used by algorithm in `/tmp/calib` folder.
