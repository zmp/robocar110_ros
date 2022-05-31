# RC110 service configuration file

# Default arguments for roslaunch.
RC110_ARGS="joy_type:=elecom"

# Change to your launch command, if you created another one.
RC110_LAUNCH_COMMAND="roslaunch --wait --skip-log-check rc110_system robot.launch ${RC110_ARGS}"

# Map name for SLAM and navigation
RC110_MAP_NAME=map