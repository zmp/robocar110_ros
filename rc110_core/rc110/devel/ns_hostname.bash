# Nodes namespace to ignore from sync.
export RC110_IGNORE=$(hostname | tr - _)  # hostname allows only hyphen, ros allows only underscore

# RoboCar name is taken either from "rc" variable or from the hostname.
export RC110_NAME=${rc:-${RC110_IGNORE}}

# ROS variable for default namespace. Leading slash prevents ambiguities.
export ROS_NAMESPACE=/${RC110_NAME}

# ROS hostname of current machine, used in zeroconf.
export ROS_HOSTNAME=$(hostname).local

# ROS master URI of current machine.
export ROS_MASTER_URI=http://${ROS_HOSTNAME}:11311
