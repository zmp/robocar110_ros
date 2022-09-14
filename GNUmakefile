#
# Convenient script to use with make on Linux.
# To check the commands list, input: make<space><tab>
#
include mk/common.mk

deps_paths  := rc110_core rc110_robot   # dependencies for robot nodes
build_nodes := rc110_rviz rc110_system  # robot nodes

define help_text

GNUmakefile provides the following targets for make:

	help                      Show this messsage
	version                   Show current versions of source and packages
	ros-install               ROS installation
	ros-source                Add ros environment to .bashrc (so commands like rostopic worked)

	init-deps-offline         Offline deps initialization, if github files download gives error (apt still needed)
	clean-deps                Clean any previous rosdep configuration in /etc/ros/rosdep/
	deps                      Install dependencies for robot  [force=on]
	deps-remote               Install dependencies for remote PC
	deps-%                    Install dependencies for %

	build (default)           Build main robot nodes
	remote                    Build nodes for remote PC
	%                         Build %
	package                   Create deb packages in build/ directory
	install                   Install the packages to system
	self                      Create "run" installation file
	clean                     Clean build and package files

	start                     Restart RoboCar nodes from system
	stop                      Stop RoboCar nodes from system
	status                    Status of system RoboCar nodes
	status-core               Status of system roscore
	list-masters              List all ros masters in the network

	run                       Run rc110_robot from build (Don't forget to stop system nodes first!)
	run-%                     Run % node from build
	show                      Show general RViz
	show-%                    Show % RViz

	sync                      Synchronize nodes of network in a separate terminal
	node-manager              Show Node Manager
	run-teleop                Run joystick [rc=rc_SN name=joy device=js0 joy_type=elecom|ps5|logicool]
	mouse-teleop              Mouse instead of joystick [rc=rc_SN]
	run-model                 Run additional gazebo model [id=123abc]

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
	JetPack (deb): $$(apt policy nvidia-jetpack 2>/dev/null | grep Installed | awk '{print $$2}') \n\
	Base Driver:   $$(apt policy rc-system 2>/dev/null | grep Installed | awk '{print $$2}') \n\
	ROS Nodes:     $$(apt policy ros-${ROS_DISTRO}-rc110 2>/dev/null | grep Installed | awk '{print $$2}') \n\
	Source Code:   $$(grep -oPm1 '(?<=<version>)[^<]+' rc110_core/rc110/package.xml) "

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
	sudo apt-get install -qq $(pythonN)-rosdep
endif
ifeq (,$(wildcard /etc/ros/rosdep/sources.list.d/20-default.list))
	sudo rosdep init
	$(eval lists_updated:=true)
endif
ifeq (,$(if ${force},,$(wildcard /etc/ros/rosdep/sources.list.d/11-robocar.list)))
	sudo cp -rf scripts/rosdep_custom/config /etc/ros/rosdep/
	sudo cp scripts/rosdep_custom/11-robocar.list /etc/ros/rosdep/sources.list.d/
	$(eval lists_updated:=true)
endif
	$(if ${lists_updated},rosdep update --rosdistro=${ROS_DISTRO})

# Install rosdep offline
init-deps-offline:
	sudo ./scripts/rosdep_offline/install
	export ROSDISTRO_INDEX_URL=file:///etc/ros/rosdep/config/index-v4.yaml

	rosdep update --rosdistro=${ROS_DISTRO}

# Clean rosdep configs
clean-deps:
	sudo rm -rf /etc/ros/rosdep

# Install robot dependencies.
deps: init-deps
	source /opt/ros/${ROS_DISTRO}/setup.bash
	rosdep install -iry --from-paths ${deps_paths}

# Install remote dependencies.
deps-remote: deps_paths := rc110_core
deps-remote: deps

# Init catkin workspace.
init:
ifeq (,$(shell which catkin))
	sudo apt-get install -qq $(pythonN)-catkin-tools
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
	    \nIf you want to have usual workspace, please, put robocar110_ros to src/\
	    \n\033[0m"
	catkin config --init --source-space . --log-space .catkin_tools/logs
  endif
endif

# Build robot nodes.
build: init
	@source /opt/ros/${ROS_DISTRO}/setup.bash
	$(call build,${build_nodes},${cmake_flags})

# Build remote nodes
remote: build_nodes := rc110_rviz rc110_teleop
remote: build

# Package robot nodes in build directory.
package: init
	@source /opt/ros/${ROS_DISTRO}/setup.bash
	$(call build,${build_nodes},${cmake_flags} -DCATKIN_BUILD_BINARY_PACKAGE=ON)

	# find core nodes with dependencies
	nodes=$$(catkin build -n ${build_nodes} | sed -n -e '/Packages to be built/,/Total packages/{//!p;}' | sed -e 's/- \(.*\)(catkin)/\1/')

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
	root_dir=$$(pwd)
	source /opt/ros/${ROS_DISTRO}/setup.bash
	cd $$(catkin locate --build)
	$${root_dir}/scripts/install_rc110

# Self-extracting archive with core packages.
self: package
ifeq (,$(shell which makeself))
	sudo apt-get install -qq makeself
endif

	root_dir=$$(pwd)
	source /opt/ros/${ROS_DISTRO}/setup.bash
	cd $$(catkin locate --build)

	file=$$(ls *rc110*.deb | head -1)
	version=$$(dpkg -f $${file} Version)
	arch=$$(dpkg -f $${file} Architecture)
	codename=$$(. /etc/os-release; echo $${UBUNTU_CODENAME})

	rm -rf stage; mkdir stage
	mv *.deb stage/
	cp $${root_dir}/scripts/install* stage/
	cp $${root_dir}/scripts/get_ros_distro stage/

	makeself stage rc110_robot_$${version}_$${arch}~$${codename}.run "package" ./install

# Clean all.
clean:
	source /opt/ros/${ROS_DISTRO}/setup.bash
	catkin clean -y

# Run nodes built from source.
run:
	source $$(catkin locate rc110)/host_setup.bash
	roslaunch --wait rc110_system robot.launch $${RC110_ARGS} $(ros_args)

# Roscore multimaster nodes synchronization in a separate terminal (may reduce launch time).
sync:
	source $$(catkin locate rc110)/host_setup.bash
	wait

# Run multimaster node manager.
node-manager:
	source $$(catkin locate rc110)/host_setup.bash
	roslaunch --wait rc110 node_manager.launch

# == additional targets ==

include mk/subdirs.mk

# == shortcuts ==

show: show-rviz
