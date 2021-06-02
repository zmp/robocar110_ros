include ../../mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF


init:
	cd ../.. && $(MAKE) init

deps: init
	$(call source)
	rosdep install -iry --from-paths .

all: init
	$(call source)
	catkin build --this --cmake-args ${cmake_flags}

run:
	source ../../../../devel/setup.bash
	roslaunch --screen rc110_video_server main.launch

show:
	gst-launch-1.0 rtspsrc latency=0 location=rtsp://127.0.0.1:8554/front \
		! rtph265depay ! h265parse ! queue ! omxh265dec ! nvoverlaysink sync=false async=false
	# ------------------------------------------------------------------------------
	# Press Ctrl+C to exit full screen.

monitor:
	$(MAKE) env -C ../..
	source ../../../../env.sh

	if [[ $$ROS_MASTER_URI =~ http://(.*):[0-9]+ ]]; then
		ip=$${BASH_REMATCH[1]}
		gst-launch-1.0 rtspsrc latency=0 location=rtsp://$$ip:8554/front \
			! rtph265depay ! h265parse ! queue ! avdec_h265 ! fpsdisplaysink sync=false async=false
	else
		echo "Unable to parse ROS_MASTER_URI"
	fi
	# ------------------------------------------------------------------------------

# Just for local testing without RTSP
test-show:
	gst-launch-1.0 -v v4l2src device=/dev/video0 \
		! 'video/x-raw, format=(string)UYVY, framerate=(fraction)60/1, width=(int)1920, height=(int)1080' \
		! xvimagesink
	# ------------------------------------------------------------------------------
