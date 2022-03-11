export RC110_NAME=$(hostname | tr - _)  # hostname allows only hyphen, ros allows only underscore
export ROS_NAMESPACE=/${RC110_NAME}
export ROS_HOSTNAME=$(hostname).local
export ROS_MASTER_URI=http://${ROS_HOSTNAME}:11311