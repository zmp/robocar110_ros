.ONESHELL:
.DEFAULT_GOAL := build
.SHELLFLAGS := -ec
SHELL := /bin/bash

cmake_flags := -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=Release
ws_path := $(shell echo $${PWD/\/src\/*/})
mk_path := $(dir $(lastword $(MAKEFILE_LIST)))
ROS_DISTRO ?= $(shell ${mk_path}../scripts/get_ros_distro)
ROS_OS ?= ubuntu:jammy

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

# See "build" usage.
define plain_build
	cd "${ws_path}"
	shutup="--allow-overriding $$(colcon list -n --packages-up-to ${1})"
	colcon build --packages-up-to ${1} $${shutup} ${if $2,$2,--symlink-install} --cmake-args ${3}

endef  # empty line at the end helps readability

# Faster build using all cores on tegra, by default with colcon --symlink-install.
# Usage
# 	$(call build,node1 node2,colcon1 colcon2,cmake1 cmake2)
ifeq (,$(shell command -v nvpmodel))
build = $(call plain_build,${1},${2},${3})  # build on non-tegra machine
else
define build
(
	source ${mk_path}../scripts/tegra_maxn.sh
	$(call plain_build,${1},${2},${3})
)
endef
endif