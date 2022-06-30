script_path=$(realpath "$(dirname "${BASH_SOURCE[0]}")")

# Start ros multimaster based on hostname, if it wasn't started yet.
source ${script_path}/host_setup.bash

# If host is not rc110 robot,
if [[ ${RC110_HOST_ID} != ${RC110_PREFIX}* ]]
then
	# Setup network once again basing on a discovered robot name.
	rc=${rc:-$(rosrun rc110_selector rc110_selector _timeout:=5000)}

	if [ -z "${rc}" ]; then
		>&2 echo -e "\nFailed to find a robot.\n"
		exit 1
	fi

	source $(catkin locate rc110)/devel/rc_env.bash
fi