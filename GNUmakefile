#
# Convenient script to use with make on Linux.
# To check the commands list, input: make<space><tab>
#
include mk/common.mk

cmake_flags := -DCATKIN_ENABLE_TESTING=OFF
main_nodes := rc110_system rc110_rviz

define help_text

GNUmakefile provides the following targets for make:

	version                   Show current versions of source and packages
	ros-install               ROS installation
	ros-source                Add ros environment to .bashrc (so commands like rostopic worked)

	init-deps-offline         Offline deps initialization, if github files download gives error (apt still needed)
	clean-deps                Clean any previous rosdep configuration in /etc/ros/rosdep/
	deps                      Install dependencies for rc110_core
	deps-%                    Install dependencies for %

	all (empty)               Build rc110_core
	%                         Build %
	package                   Create deb packages in build/ directory
	install                   Install the packages to system
	self                      Create "run" installation file
	clean                     Clean build and package files

	start                     Restart RoboCar nodes from system
	stop                      Stop RoboCar nodes from system
	run                       Run rc110_core from build (Don't forget to stop system nodes first!)
	run-%                     Run % node from build
	show                      Show general RViz on robot
	show-%                    Show % RViz on robot

	env                       Create env.sh file for connection to remote PC (Needs to be edited!)
	monitor                   Show general RViz on remote PC
	monitor-%                 Show % RViz on remote PC
	remote-teleop             Run joystick connected to remote PC [device:=js0 joy_type:=elecom|ps5|logicool]
	mouse-teleop              Mouse instead of joystick on remote PC

	save-map-he               Save Hector SLAM map [map_name=map]
	save-map-cg               Save Cartographer SLAM map [map_name=map map_resolution=0.025]
	select-map                Select map for navigation [map_name=map]

endef

# == targets ==

export help_text
help:
	@echo -e "\033[1;36m$${help_text}\033[0m"

# Show versions
version:
	@echo -e "\
	Tegra: $$(cat /etc/nv_tegra_release 2> /dev/null) \n\
	Last JetPack: $$(apt-cache show nvidia-jetpack 2> /dev/null | grep Version | awk 'NR==1{print $$2}') \n\
	Base Driver:  $$(dpkg -s rc-system 2> /dev/null | grep Version | awk '{print $$2}') \n\
	ROS Nodes:    $$(dpkg -s ros-${ROS_DISTRO}-rc110-common 2> /dev/null | grep Version | awk '{print $$2}') \n\
	Source Code:  $$(grep -oPm1 '(?<=<version>)[^<]+' rc110_core/rc110_common/package.xml) "

# Shortcut for ROS installation.
ros-install:
	./scripts/install_ros

# Add ros environment to .bashrc.
ros-source:
	@
ifeq (,$(shell grep -q "source /opt/ros" ~/.bashrc))
	echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
	echo "ROS sourcing is added. Please, restart the shell to apply."
else
	echo "ROS sourcing exists already."
endif

# Install rosdep and get packages list.
init-deps:
ifeq (,$(shell which rosdep))
	sudo apt-get install -qq python-rosdep
endif
ifeq (,$(wildcard /etc/ros/rosdep/sources.list.d/20-default.list))
	sudo rosdep init
	rosdep update --rosdistro=${ROS_DISTRO}
endif

# Install rosdep offline
init-deps-offline:
	sudo scripts/rosdep_offline/install
	export ROSDISTRO_INDEX_URL=file:///etc/ros/rosdep/config/index-v4.yaml

	rosdep update --rosdistro=${ROS_DISTRO}

# Clean rosdep configs
clean-deps:
	sudo rm -rf /etc/ros/rosdep

# Install core dependencies.
deps: init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths rc110_core rc110_robot

# Init catkin workspace.
init:
ifeq (,$(shell which catkin))
	sudo apt-get install -qq python-catkin-tools
endif
ifeq (src,$(notdir $(abspath ..)))
  ifeq (,$(wildcard ../../.catkin_tools))
	cd ../..
	catkin config --init --log-space .catkin_tools/logs
  endif
else
  ifeq (,$(wildcard .catkin_tools))
	@echo -e "\033[1;31m\
	    \nWarning: Catkin workspace will be created in the current directory!\
	    \nCreate and use rc110_contrib/ directory for new nodes.\
	    \nIf you want to have usual workspace, please, put robocar110_ros to src/\
	    \n\033[0m"
	catkin config --init --source-space . --log-space .catkin_tools/logs
  endif
endif

# Build core nodes.
all: init
	@source /opt/ros/${ROS_DISTRO}/setup.bash
	$(call build,${main_nodes},${cmake_flags})

# Package core nodes in build directory.
package: init
	@source /opt/ros/${ROS_DISTRO}/setup.bash
	$(call build,${main_nodes},${cmake_flags} -DCATKIN_BUILD_BINARY_PACKAGE=ON)

	# find core nodes with dependencies
	nodes=$$(catkin build -n ${main_nodes} | sed -n -e '/Packages to be built/,/Total packages/{//!p;}' | sed -e 's/- \(.*\)(catkin)/\1/')

	cd $$(catkin locate --build)
	rm -f *.deb
	for node in $$nodes
	do
		(  # parallel run
		cd "$$node"
			$(MAKE) package >/dev/null
			mv *.deb ../
			echo "Package created: $$node"
		cd ..
		) &
	done
	wait

# Install core packages to system folder.
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

	makeself stage rc110_robot_$${version}.run "package" ./install

# Clean all.
clean:
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin clean -y

# Environment variables for remote access.
env:
ifeq (,$(wildcard ../../env.sh))
	cp mk/env_template.sh ../../env.sh
endif

# Prepare variables for run-* targets.
init-run:
ifeq (,$(wildcard ~/.config/rc110/service.conf))
	mkdir -p ~/.config/rc110
	cp rc110_robot/rc110_system/deb/service_template.conf ~/.config/rc110/service.conf
endif

# Run nodes built from source.
run: init-run
	roscore &>/dev/null &
	source ../../devel/setup.bash
	source ~/.config/rc110/service.conf
	eval "$$RC110_LAUNCH_COMMAND"

# Run only joystick node on remote PC.
remote-teleop: env
	source ../../env.sh
	$(MAKE) run -C rc110_core/rc110_teleop joy_topic:=joy_remote

# Manipulation with mouse on remote PC.
mouse-teleop: env
	source ../../env.sh
	$(MAKE) mouse -C rc110_core/rc110_teleop

# == additional targets ==

include mk/subdirs.mk

# == shortcuts ==

show: show-rviz
monitor: monitor-rviz
