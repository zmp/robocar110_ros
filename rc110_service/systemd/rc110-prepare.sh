source /opt/ros/@ROS_DISTRO@/setup.bash

# Give ifconfig 5 seconds to get the ip address.
for i in {1..5}; do
    export ROS_IP=$(ifconfig "${RC110_INTERFACE}" | grep 'inet ' | awk '{print $2}')

    if [ -n "$ROS_IP" ]; then
        break;
    fi
    sleep 1
done

if [ -z "$ROS_IP" ]; then
    echo "Failed command: ifconfig ${RC110_INTERFACE}" >&2
    unset ROS_IP
fi
