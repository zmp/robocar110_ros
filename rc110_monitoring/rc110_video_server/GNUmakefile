include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


deps:
	$(MAKE) -C ../.. init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths .

build:
	$(MAKE) -C ../.. init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin build --this --cmake-args ${cmake_flags}

run:
	source $$(catkin locate rc110)/auto_setup.bash
	roslaunch --wait rc110_video_server main.launch $(ros_args)

show:
	source $$(catkin locate rc110)/auto_setup.bash
	host=$$(echo $${rc} | tr _ -)
	if [ "$${host}" != "$$(hostname)" ]
	then # remote
		decoder_sync='avdec_h265 ! $(if $(overlay),fpsdisplaysink sync=false async=false,xvimagesink)'
	else # local
		decoder_sync='omxh265dec ! nvoverlaysink sync=false async=false'
	fi
	gst-launch-1.0 rtspsrc latency=0 location=rtsp://$${host}.local:8554/front \
			! rtph265depay ! h265parse ! queue ! $${decoder_sync}
	# ------------------------------------------------------------------------------
	# Press Ctrl+C to exit full screen.
	#

# Just for local testing without RTSP
test-show:
	gst-launch-1.0 -v v4l2src device=/dev/video0 \
		! 'video/x-raw, format=(string)UYVY, framerate=(fraction)60/1, width=(int)1920, height=(int)1080' \
		! xvimagesink
	# ------------------------------------------------------------------------------
