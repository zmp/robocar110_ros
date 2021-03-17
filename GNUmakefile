# Convenient script to use with make on Linux.

.DEFAULT_GOAL := all
.ONESHELL:
.SHELLFLAGS := -ec
SHELL := /bin/bash

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF
main_nodes := rc110_service rc110_rviz

ROS_DISTRO ?= melodic

define source
	source /opt/ros/${ROS_DISTRO}/setup.bash
endef

define env_content
# Please, adjust the ips below to match you real ips.

export ROS_MASTER_URI=http://192.168.110.5:11311
export ROS_IP=192.168.110.2
echo
echo "../../env.sh is configued as: ROS_MASTER_URI=$$ROS_MASTER_URI, ROS_IP=$$ROS_IP"
echo
endef


ros-source:
	@
ifeq (,$(shell grep -q "source /opt/ros" ~/.bashrc))
	echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
	echo "ROS sourcing is added. Please, restart the shell to apply."
else
	echo "ROS sourcing exists already."
endif

init:
ifneq (0, $(shell catkin locate &>/dev/null; echo $$?))
	sudo apt-get install -qq python-catkin-tools
	cd ../..
	catkin init
endif
ifeq (,$(wildcard /etc/ros/rosdep/sources.list.d/20-default.list))
	sudo apt-get install -qq python-rosdep
	sudo rosdep init
	rosdep update --rosdistro=${ROS_DISTRO}
endif

deps: init
	$(call source)
	rosdep install -iry --from-paths rc110_*

all: init
	$(call source)
	catkin build ${main_nodes} --cmake-args ${cmake_flags}

package: init
	$(call source)
	catkin build ${main_nodes} --cmake-args ${cmake_flags} -DCATKIN_BUILD_BINARY_PACKAGE=1

	function check_make_target {
		output=$$(make -n "$$1" 2>&1 | head -1)
		[[ "$$output" != *"No rule to make target"* ]]
	}

	cd $$(catkin locate --build)
	rm -f *.deb
	for d in */
	do
		(  # make package in parallel
		pushd "$$d"
			if check_make_target package; then
				$(MAKE) package
				mv *.deb ../
			fi
		popd
		) &
	done
	wait

install: package
	$(call source)
	cd $$(catkin locate --build)
	sudo apt-get install --reinstall ./*.deb
	systemctl --user daemon-reload  # automatic files reload - it does not work from postinst, as root runs postinst

deps-rviz: init
	$(call source)
	rosdep install -iry --from-paths rc110_rviz --skip-keys=rc110_msgs

rviz: init
	$(call source)
	catkin build rc110_rviz --cmake-args ${cmake_flags}

show:
	source ../../devel/setup.bash
	roslaunch rc110_rviz rviz.launch

export env_content
env:
ifeq (,$(wildcard ../../env.sh))
	echo "$$env_content" > ../../env.sh
endif

remote-show: env
	source ../../env.sh
	$(MAKE) show

remote-drive: env
	source ../../env.sh
	source ../../devel/setup.bash
	rosrun joy joy_node __name:=joy_node_remote

clean:
	$(call source)
	catkin clean -y


# advanced nodes
adv_nodes := $(subst _,-,$(subst rc110_,,$(sort $(notdir $(wildcard advanced/rc110_*)))))
deps_adv_nodes := $(addprefix deps-,$(adv_nodes))
run_adv_nodes := $(addprefix run-,$(adv_nodes))
show_adv_nodes := $(addprefix show-,$(adv_nodes))

$(adv_nodes):
	cd advanced/$(subst -,_,$(addprefix rc110_,$@)) && $(MAKE)

$(deps_adv_nodes):
	cd advanced/$(subst -,_,$(addprefix rc110_,$(subst deps-,,$@))) && $(MAKE) deps

$(run_adv_nodes):
	cd advanced/$(subst -,_,$(addprefix rc110_,$(subst run-,,$@))) && $(MAKE) run

$(show_adv_nodes):
	cd advanced/$(subst -,_,$(addprefix rc110_,$(subst show-,,$@))) && $(MAKE) show
