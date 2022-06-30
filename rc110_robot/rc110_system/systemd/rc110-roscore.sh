#!/bin/bash

source /opt/ros/@ROS_DISTRO@/rc_setup.bash

end=$((SECONDS+5))
while [ $SECONDS -lt $end ]
do
	if output=$(host ${ROS_HOSTNAME})
	then
		echo "${output}"
		break;
	fi
done

echo "Starting multimaster..."
roslaunch rc110 multimaster.launch
