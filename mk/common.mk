.ONESHELL:
.DEFAULT_GOAL := build
.SHELLFLAGS := -ec
SHELL := /bin/bash

ROS_DISTRO ?= melodic

# Returns whether roscore is stopped.
roscore_stopped = $(shell rosnode list &>/dev/null && echo false || echo true)

# Convert makefile variables to roslaunch arguments.
ros_args = $(shell \
  with_vars=$$(grep ' \-- ' <<< "$(MAKEFLAGS)" || true); \
  vars=$${with_vars\#* -- }; \
  echo $${vars//'='/':='} \
)
# Explanation:
# 1. get only flags containing --
# 2. remove flags before --
# 3. replace = with :=
#
# Same can be done by make only:
# ros_args = $(and $(findstring $() -- $(),$(MAKEFLAGS)),$(subst =,:=,$(subst ?, ,$(lastword $(subst ?--?, ,$(subst $() $(),?,$(MAKEFLAGS)))))))
#

# Faster build using all cores on tegra.
define build
(
	if [ -x "$$(command -v nvpmodel)" ]  # if there's nvpmodel command,
	then
		last_nvpmode=$$(nvpmodel -q | sed -n 3p)   # get last mode
		if [ 0 -ne $${last_nvpmode} ]              # if it's not MaxN,
		then
			echo -e "\033[1;31m\n Using all CPU cores during build. Requires entering sudo!\n\033[0m"
			sudo nvpmodel -m 0                     # set MaxN
			function cleanup {
				sudo nvpmodel -m $${last_nvpmode}  # on exit from build, revert to the last mode
			}
			trap cleanup EXIT
		fi
	fi
	catkin build ${1} --cmake-args ${2}
)
endef
