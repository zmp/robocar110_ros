.ONESHELL:
.DEFAULT_GOAL := build
.SHELLFLAGS := -ec
SHELL := /bin/bash

mk_path := $(dir $(lastword $(MAKEFILE_LIST)))
ROS_DISTRO ?= $(shell ${mk_path}/../scripts/get_ros_distro)
pythonN := $(if $(filter melodic,${ROS_DISTRO}),python,python3)

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
		eval $$(cat /var/lib/nvpmodel/status | tr : =)  # get last mode as "pmode"

		if [ 0000 -ne $${pmode} ]           # if it's not MaxN,
		then
			echo -e "\033[1;31m\n Using all CPU cores during build. Requires entering sudo!\n\033[0m"

			sudo nvpmodel -m 0              # set MaxN
			function cleanup {
				sudo nvpmodel -m $${pmode}  # on exit from build, revert to the last mode
			}
			trap cleanup EXIT
		fi
	fi
	catkin build ${1} --cmake-args ${2}
)
endef
