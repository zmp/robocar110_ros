# Nodes namespace to ignore from sync. Also used on robot to initialize RC110_NAME.
export RC110_HOST_ID=$(hostname | tr - _)  # hostname allows only hyphen, ros allows only underscore

# RoboCar name used for tf prefix and simulation model ns.
export RC110_NAME=${rc:-${RC110_HOST_ID}}

# NS is taken either from "rc" or from the hostname. Leading slash prevents ambiguities.
export ROS_NAMESPACE=/${RC110_NAME}

# ROS hostname of current machine, used in zeroconf.
export ROS_HOSTNAME=$(hostname).local

# ROS master URI of current machine.
export ROS_MASTER_URI=http://${ROS_HOSTNAME}:11311

# default prefixes
export RC110_HOST_PREFIX=${RC110_HOST_PREFIX:-rc-}
export RC110_PREFIX=$(echo ${RC110_HOST_PREFIX} | tr - _)
