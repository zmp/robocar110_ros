source $(catkin locate --devel)/rc_setup.bash "$@"

# Run roscore with client, if it's not already running.
if ! rosnode list &>/dev/null
then
	trap 'kill $(jobs -p); wait' EXIT  # kill background jobs on exit and wait for them to stop

	roslaunch rc110 multimaster.launch &
fi
