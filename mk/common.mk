.ONESHELL:
.DEFAULT_GOAL := build
.SHELLFLAGS := -ec
SHELL := /bin/bash

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF
mk_path := $(dir $(lastword $(MAKEFILE_LIST)))
ROS_DISTRO ?= $(shell ${mk_path}../scripts/get_ros_distro)
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

# Faster build using all cores on tegra, by default with colcon --symlink-install.
# Usage
# 	$(call build,node1 node2,colcon1 colcon2,cmake1 cmake2)
ifeq (,$(shell command -v nvpmodel))
build = catkin build ${1} --cmake-args ${2}  # build on non-tegra machine
else
define build
(
	catkin build ${1} --cmake-args ${2}
)
endef
endif
