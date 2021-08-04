#
# Convenient script to use with make on Linux.
# To check the commands list, input: make<space><tab>
#
include mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF
main_nodes := rc110_system rc110_rviz


# == targets ==

# Add ros environment to .bashrc.
ros-source:
	@
ifeq (,$(shell grep -q "source /opt/ros" ~/.bashrc))
	echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
	echo "ROS sourcing is added. Please, restart the shell to apply."
else
	echo "ROS sourcing exists already."
endif

# Install catkin and rosdep.
init:
ifeq (,$(shell which catkin))
	sudo apt-get install -qq python-catkin-tools
	cd ../..
	catkin init
endif
ifeq (,$(wildcard /etc/ros/rosdep/sources.list.d/20-default.list))
	sudo apt-get install -qq python-rosdep
	sudo rosdep init
	rosdep update --rosdistro=${ROS_DISTRO}
endif

# Install core dependencies.
deps: init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths rc110_core

# Build all.
all: init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	$(call build,${main_nodes},${cmake_flags})

# Make packages in build directory.
package: init
	source /opt/ros/${ROS_DISTRO}/setup.bash
	$(call build,${main_nodes},${cmake_flags} -DCATKIN_BUILD_BINARY_PACKAGE=1)

	function check_make_target {
		output=$$(make -n "$$1" -C "$$2" 2>&1 || true | head -1)
		[[ "$$output" != *"No rule to make target"* ]]
	}

	cd $$(catkin locate --build)
	rm -f *.deb
	for d in */
	do
		(  # parallel run
		if check_make_target package "$$d"
		then
			cd "$$d"
				$(MAKE) package >/dev/null
				mv *.deb ../
				echo "Package created: $$d"
			cd ..
		fi
		) &
	done
	wait

# Install packages to system folder.
install: package
	source /opt/ros/${ROS_DISTRO}/setup.bash
	cd $$(catkin locate --build)
	sudo dpkg --configure -a  # resolves "Internal Error, No file name for"
	sudo apt-get install -qq --allow-downgrades --reinstall ./*.deb
	systemctl --user daemon-reload  # automatic files reload - it does not work from postinst, as root runs postinst

# Self-extracting archive with core packages.
self: package
ifeq (,$(shell which makeself))
	sudo apt-get install -qq makeself
endif

	root_dir=$$(pwd)
	source /opt/ros/${ROS_DISTRO}/setup.bash
	cd $$(catkin locate --build)
	
	version=$$(dpkg-deb -f $$(ls *rc110*.deb | head -1) Version)
	rm -rf stage; mkdir stage
	mv *.deb stage/
	cp $${root_dir}/scripts/install* stage/

	makeself stage rc110_core_$${version}.run "package" ./install

# Environment variables for remote access.
env:
ifeq (,$(wildcard ../../env.sh))
	cp mk/env_template.sh ../../env.sh
endif

# Run nodes built from source.
run:
	systemctl --user start rc110-roscore
	source ../../devel/setup.bash
	source ~/.config/rc110/service.conf
	eval "$$RC110_LAUNCH_COMMAND"

# Run only joystick node on remote PC.
remote-joy: env
	source ../../devel/setup.bash
	source ../../env.sh
	roslaunch rc110_launch remote_joy.launch

# Clean all.
clean:
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin clean -y


# == additional targets ==

include mk/subdirs.mk

# == shortcuts ==

show: show-rviz
monitor: monitor-rviz
