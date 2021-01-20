.DEFAULT_GOAL := all
.ONESHELL:
.SHELLFLAGS := -ec
SHELL := /bin/bash

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF

define source
	source /opt/ros/melodic/setup.bash
endef


ros-source:
	@
ifeq (,$(shell grep -q "source /opt/ros" ~/.bashrc))
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
	echo "ROS sourcing is added. Please, restart the shell to apply."
else
	echo "ROS sourcing exists already."
endif

init:
	sudo apt-get install -y -q python-catkin-tools python-rosdep
	cd ../..
	catkin init

	sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list
	sudo rosdep init
	rosdep update --rosdistro=$${ROS_DISTRO}

deps:
	$(call source)
	rosdep install -iry --from-paths .

all:
	$(call source)
	catkin build ${cmake_flags}

package:
	$(call source)
	catkin build ${cmake_flags} -DCATKIN_BUILD_BINARY_PACKAGE=1

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

deps-rviz:
	$(call source)
	rosdep install -iry --from-paths rc110_rviz --skip-keys=rc110_msgs

rviz:
	$(call source)
	catkin build rc110_rviz ${cmake_flags}

clean:
	$(call source)
	catkin clean -y

config:
	sudo ./rc110_launch/deb/postinst
