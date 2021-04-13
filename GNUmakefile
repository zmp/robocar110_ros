#
# Convenient script to use with make on Linux.
# To check the commands list, input: make<space><tab>
#
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
	rosdep install -iry --from-paths rc110_core rc110_navigation/rc110_behavior

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
	sudo apt-get install -qq --allow-downgrades --reinstall ./*.deb
	systemctl --user daemon-reload  # automatic files reload - it does not work from postinst, as root runs postinst

export env_content
env:
ifeq (,$(wildcard ../../env.sh))
	cp mk/env_template.sh ../../env.sh
endif

run:
	systemctl --user start rc110-roscore
	source ../../devel/setup.bash
	. ~/.config/rc110/service.conf
	eval "$$RC110_LAUNCH_COMMAND"

remote-joy: env
	$(call source)
	source ../../env.sh
	rosrun joy joy_node __name:=joy_node_remote

clean:
	$(call source)
	catkin clean -y


# additional targets
include mk/subdirs.mk

# shortcuts
show: show-rviz
monitor: monitor-rviz