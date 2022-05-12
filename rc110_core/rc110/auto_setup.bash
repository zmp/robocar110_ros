script_path=$(realpath "$(dirname "${BASH_SOURCE[0]}")")

# Start ros multimaster based on hostname, if it wasn't started yet.
source ${script_path}/host_setup.bash

# Set nodes waiting timeout [seconds], and setup network once again basing on a discovered robot name.
RC110_TIMEOUT=5
i=0;
while true
do
	# RoboCar name is taken from the first robot in node list.
	br=''$'(\n|$)'''  # line break or string end
	rc=${rc:-$([[ $(rosnode list 2>/dev/null) =~ /([a-zA-Z0-9_]*)/tf_publisher${br} ]]; echo ${BASH_REMATCH[1]})}
	if [ -n "${rc}" ]; then break; fi

	echo "."

	if [ $((i / 1000)) -ge ${RC110_TIMEOUT} ]
	then
		>&2 echo "Failed to find a robot."
		exit 1
	fi
	sleep 0.5;
	(( i += 500 ))
done

source $(catkin locate rc110)/devel/ns_hostname.bash
