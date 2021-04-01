# Convenient script to use with make on Linux.
include mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF
main_nodes := rc110_system rc110_rviz


# targets

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
	$(call build,${main_nodes},${cmake_flags})

package: init
	$(call source)
	$(call build,${main_nodes},${cmake_flags} -DCATKIN_BUILD_BINARY_PACKAGE=1)

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

export env_content
env:
ifeq (,$(wildcard ../../env.sh))
	cp mk/env_template.sh ../../env.sh
endif

remote-joy: env
	$(call source)
	source ../../env.sh
	rosrun joy joy_node __name:=joy_node_remote

clean:
	$(call source)
	catkin clean -y


# advanced nodes
adv_node_dirs := $(subst /GNUmakefile,,$(wildcard rc110_*/rc110_*/GNUmakefile))  # all nodes that contain GNUmakefile
adv_nodes := $(subst _,-,$(subst rc110_,,$(sort $(notdir $(adv_node_dirs)))))    # get their names

$(adv_nodes):
	$(MAKE) -C $(wildcard rc110_*/$(subst -,_,$(addprefix rc110_,$@)))

deps_adv_nodes := $(addprefix deps-,$(adv_nodes))
$(deps_adv_nodes):
	$(MAKE) deps -C $(wildcard rc110_*/$(subst -,_,$(addprefix rc110_,$(subst deps-,,$@))))

run_adv_nodes := $(addprefix run-,$(adv_nodes))
$(run_adv_nodes):
	$(MAKE) run -C $(wildcard rc110_*/$(subst -,_,$(addprefix rc110_,$(subst run-,,$@))))

show_adv_nodes := $(addprefix show-,$(adv_nodes))
$(show_adv_nodes):
	$(MAKE) show -C $(wildcard rc110_*/$(subst -,_,$(addprefix rc110_,$(subst show-,,$@))))

monitor_adv_nodes := $(addprefix monitor-,$(adv_nodes))
$(monitor_adv_nodes):
	$(MAKE) monitor -C $(wildcard rc110_*/$(subst -,_,$(addprefix rc110_,$(subst monitor-,,$@))))


# convenient shortcuts
show: show-rviz
monitor: monitor-rviz

# additional targets
include rc110_core/rc110_launch/mk/camera_calibration.mk
