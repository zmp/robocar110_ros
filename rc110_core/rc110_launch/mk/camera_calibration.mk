camera_types := front rear

$(addprefix camera-calibration-file-,$(camera_types)):
	camera_type=$(subst camera-calibration-file-,,$@)

	tar -xzf /tmp/calibrationdata.tar.gz -C /tmp --one-top-level=calib  # creates ost.yaml file
	sed -i 's/camera_model: plumb_bob/distortion_model: plumb_bob/g' /tmp/calib/ost.yaml  # replaces camera_model with distortion_model

	mkdir -p ~/.ros/camera_info
	mv /tmp/calib/ost.yaml ~/.ros/camera_info/$${camera_type}_camera.yaml
