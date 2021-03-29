# note on monocular camera's calibration #
***

The ~/image_raw messages contain the distorted images of the front (or optional rear) camera.

Intrinsic calibration needs to be done to get the intrinsic parameters of the camera(s); also to obtain distortion coefficients to undistort the images.

Follow the instruction from [HERE](http://wiki.ros.org/camera_calibration), calibrated data will be saved into */tmp/calibrationdata.tar.gz*

```bash
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.35 image:=/front_camera/image_raw camera:=/front_camera
# for calibration board of different size, please change the value of --size and --square
```

Suppose you are doing calibration for front camera of rc110, please do the following steps:

- copy the yaml to predefined place
```bash
tar -xvzf /tmp/calibrationdata.tar.gz
mkdir -p $HOME/.config/rc110/calibration
mv ost.yaml $HOME/.config/rc110/calibration/front_camera.yaml
# for rear camera, after rear camera's calibration, just change front_camera.yaml to rear_camera.yaml in the above command line.
```
- modify the content of the yaml file
```text
change "camera_name: narrow_stereo" to "camera_name: head_camera"

change "camera_model: plumb_bob" to "distortion_model: plumb_bob"

# or use the following command lines
sed -i 's/camera_name: narrow_stereo/camera_name: head_camera/g' $HOME/.config/rc110/calibration/front_camera.yaml
sed -i 's/camera_model: plumb_bob/distortion_model: plumb_bob/g' $HOME/.config/rc110/calibration/front_camera.yaml
```
- It is recommended to use ~/image_rect_color (color undistorted image message) or ~/image_rect (monochrome undistorted image message) as inputs for your awesome computer vision applications.
